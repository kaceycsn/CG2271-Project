#include "MKL25Z4.h" 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h" 
#define RED_LED 18 //PortB Pin 18
#define GREEN_LED 19 //PortB Pin 19
#define BLUE_LED 1 //PortD Pin 1
#define MASK(x) (1 << (x))

#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128

#define Q_SIZE 32

#define FORWARD 1
#define BACKWARDS 0
#define LEFT 2
#define RIGHT 3

#define LEFT_FORWARD 0 // PTB0 TPM1_CH0 (M1 & M3)
#define LEFT_BACK 1 // PTB1 TPM1_CH1 (M1 & M3)
#define RIGHT_FORWARD 2 // PTB2 TPM2_CH0 (M2 & M4)
#define RIGHT_BACK 3 //PTB3 TPM2_CH1 (M2 & M4)
#define DUTY_CYCLE 0x1D4C // 7500 (50hz)
 
#define R2 0x01
#define L2 0x02
#define SQUARE 0x03
#define TRIANGLE 0x04
#define CROSS 0x05
#define CIRCLE 0x06
#define RJOYSTICK_RIGHT 0x07
#define RJOYSTICK_LEFT 0x08
#define LJOYSTICK_UP 0x10
#define LJOYSTICK_DOWN 0x09

 typedef enum {
  led_on,
  led_off
} led_status_t;

typedef struct { 
  unsigned char DATA[Q_SIZE];
  // Done halfway; copy the rest from lect 8 page 18
  unsigned int HEAD;
  unsigned int TAIL;
  unsigned int SIZE;
} Q_T;

Q_T tx_q, rx_q;
//volatile uint8_t rx_IRQ_data = 0x00;
volatile uint8_t rx_data[2] = {117, 122}; // initialize to neutral
volatile uint8_t rx_index = 0;

void Q_Init(Q_T *q) {
  unsigned int i;
  for(i=0; i < Q_SIZE; i++) q->DATA[i] = 0; // Initialise to 0
  q->HEAD = 0;
  q->TAIL = 0;
  q->SIZE = 0;
}

int Q_Empty(Q_T *q) {
  return q->SIZE == 0;
}

int Q_Full(Q_T *q) {
  return q->SIZE == Q_SIZE;
}

int Q_Enqueue(Q_T *q, unsigned char d) {
  if(Q_Full(q)) return 0; // Queue full - Failure
  q->DATA[q->TAIL++] = d;
  q->TAIL %= Q_SIZE; // This makes the list circular
  q->SIZE++;
  return 1; // Success
}

unsigned char Q_Dequeue(Q_T *q) {
  if(Q_Empty(q)) return 0; // Nothing to dequeue
  unsigned char t = q->DATA[q->HEAD];
  q->DATA[q->HEAD++] = 0;
  q->HEAD %= Q_SIZE;
  q->SIZE--;
  return t;
}

void initLED(void) {
  //Enable CLock to PORTB and PORTD
  SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK));
  
  //Configure MUX settings 
  PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK; //clear bit 10 to 8
  PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1); //Select GPIO, from datasheet it is alternative 1, bits 10-8 are 001 so we use 1
  
  PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK; //clear bit 10 to 8
  PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1); //Select GPIO, from datasheet it is alternative 1, bits 10-8 are 001 so we use 1
  
  PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK; //clear bit 10 to 8
  PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1); //Select GPIO, from datasheet it is alternative 1, bits 10-8 are 001 so we use 1
  
  //Set Data Direction Registers for PortB and PortD
  PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));
  PTD->PDDR |= MASK(BLUE_LED);
}

void offAllLed(void) { //off all LED
  PTB->PSOR = MASK(RED_LED);
  PTB->PSOR = MASK(GREEN_LED);
  PTD->PSOR = MASK(BLUE_LED);
}

void initUART2(uint32_t baud_rate) {

    uint32_t divisor, bus_clock;
  
    SIM->SCGC4 |= SIM_SCGC4_UART2_MASK; //enable clock to UART2 module
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK; //enable clock to PORT E module

    PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK; //clear PORTE_PCR22
    PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4); //enable pin mux alternative 4, page 163, which enables UART2_TX

    PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK; //clear PORTE_PCR23
    PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4); //enable UART2_RX

    UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK)); //disable tx and rx before configuration
		bus_clock = (DEFAULT_SYSTEM_CLOCK) / 2;
    divisor = bus_clock / (baud_rate * 16); //oversampling of UART 2, 16*baud rate is most commonly used, each serial bit sampled 16 times
    UART2->BDH = UART_BDH_SBR(divisor >> 8);
    UART2->BDL = UART_BDL_SBR(divisor);

    // No parity, 8 bits, two stop bits, other settings
    UART2->C1 = 0;
    UART2->S2 = 0;
    UART2->C3 = 0;

    // Queue
    NVIC_SetPriority(UART2_IRQn, 128);
    NVIC_ClearPendingIRQ(UART2_IRQn);
    NVIC_EnableIRQ(UART2_IRQn);

    // // Enable TX and RX Interrupts
    UART2->C2 |= UART_C2_TIE_MASK | UART_C2_RIE_MASK;

    // Initialise both tx_q and rx_q
    Q_Init(&tx_q);
    Q_Init(&rx_q);

    UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK)); //Enable TX and RX
}

void initMotorPWM(void) {
    // enable clock gating for PORTB
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

    // configure mode 3 for the PWM pin operation
    // For left motors
    PORTB->PCR[LEFT_FORWARD] &= ~PORT_PCR_MUX_MASK;
    PORTB->PCR[LEFT_FORWARD] |= PORT_PCR_MUX(3);
    PORTB->PCR[LEFT_BACK] &= ~PORT_PCR_MUX_MASK;
    PORTB->PCR[LEFT_BACK] |= PORT_PCR_MUX(3);

    // For right motors
    PORTB->PCR[RIGHT_FORWARD] &= ~PORT_PCR_MUX_MASK;
    PORTB->PCR[RIGHT_FORWARD] |= PORT_PCR_MUX(3);
    PORTB->PCR[RIGHT_BACK] &= ~PORT_PCR_MUX_MASK;
    PORTB->PCR[RIGHT_BACK] |= PORT_PCR_MUX(3);

    // enable clock gating for Timer 1
    SIM->SCGC6 = (SIM_SCGC6_TPM1_MASK) | (SIM_SCGC6_TPM2_MASK);

    // select clock for TPM module
    SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // MCGFLLCLK clock or MCGPLLCLK/2 clock

    // Set Mod value 48000000 / 120 = 375000 / 7500 = 50hz
    TPM1->MOD = 7500;
    TPM2->MOD = 7500;

    // Edged aligned PWM
    // upate snc register: CMOD = 01, PS = 111 (128)
    TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
    TPM1->SC |= (TPM_SC_CMOD(1)) | (TPM_SC_PS(7)); // LPTPM counter increments on every LPTPM counter clock
    TPM1->SC &= ~(TPM_SC_CPWMS_MASK); // up counting mode

    // Same config as TPM1
    TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
    TPM2->SC |= (TPM_SC_CMOD(1)) | (TPM_SC_PS(7)); 
    TPM2->SC &= ~(TPM_SC_CPWMS_MASK);

    // Enable PWM on TPM1 Channel 0 -> PTB0
    TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

    // Enable PWM on TPM1 Channel 1 -> PTB1
    TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

    // Enable PWM on TPM2 Channel 0 -> PTB3
    TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

    // Enable PWM on TPM2 Channel 1 -> PTB4
    TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    TPM2_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

void stopMotor() {
    TPM1->MOD = 0;
    // Captured LPTPM counter value of the input modes
    TPM1_C0V = 0; // Stop Left motors Fw
    TPM1_C1V = 0; // Stop Left motors Back

    TPM2->MOD = 0;
    TPM2_C0V = 0; // Stop Right motors fw
    TPM2_C1V = 0; // Stop Right motors back
}

void forwardMotor() { // FORWARD
    TPM1->MOD = 7500;
    TPM1_C0V = DUTY_CYCLE; // Left motors forward

    TPM2->MOD = 7500;
    TPM2_C0V = DUTY_CYCLE; // Right motors forward
}

void reverseMotor() {
    TPM1->MOD = 7500;
    TPM1_C1V = DUTY_CYCLE; // Left motors reverse

    TPM2->MOD = 7500;
    TPM2_C1V = DUTY_CYCLE; // Right motors reverse
}

void leftTurn() {
    TPM1->MOD = 7500;
    TPM1_C0V = 1000; // Left motors slower

    TPM2->MOD = 7500;
    TPM2_C0V = DUTY_CYCLE; // Right motors faster
}

void rightTurn() {
    TPM1->MOD = 7500;
    TPM1_C0V = DUTY_CYCLE; // Left motors faster

    TPM2->MOD = 7500;
    TPM2_C0V = 1000; // Right motors slower
}

void rotateRight(int speed) { //RIGHT
    TPM1->MOD = 7500;
    TPM1_C1V = speed; // Left motors forward

    TPM2->MOD = 7500;
    TPM2_C0V = speed; // Right motors reverse
}

void rotateLeft(int speed) { // LEFT
    TPM1->MOD = 7500;
    TPM1_C0V = speed; // Left motors reverse

    TPM2->MOD = 7500;
    TPM2_C1V = speed; // Right motors forward
}


static void delay(volatile uint32_t nof) {
    while(nof != 0) {
        __asm("NOP");
        nof--;
    }
}

void UART2_IRQHandler(void) {
  NVIC_ClearPendingIRQ(UART2_IRQn);
  // Transmitter ready
  if(UART2->S1 & UART_S1_TDRE_MASK) {
    if(Q_Empty(&tx_q)) {
            // Queue Empty so disable interrupts
            UART2->C2 &= ~UART_C2_TIE_MASK; 
        } else {
            UART2->D = Q_Dequeue(&tx_q);
        }
  }

  // If receiver is full
  if(UART2->S1 & UART_S1_RDRF_MASK) {
        if(Q_Full(&rx_q)) {
            while(1); // TODO: Handle error
        } else {
            uint8_t byte = UART2->D;
	    rx_data[rx_index] = byte;
            rx_index = (rx_index + 1) % 2; // Alternate between index 0 and 1
        }
  }

  // Error checking
  if(UART2->S1 & (UART_S1_OR_MASK | 
                    UART_S1_NF_MASK | 
                    UART_S1_FE_MASK | 
                    UART_S1_PF_MASK)) {
    // TODO: Handle error
    // TODO: Clear Flag
    return;
  }
}

volatile _Bool is_led_running = 0;

void motor_thread(void *argument) {
    // Define neutral values and a dead zone threshold.
    const uint8_t X_NEUTRAL = 117; // Joystick neutral for forward/back
    const uint8_t Y_NEUTRAL = 122; // Joystick neutral for left/right (average of 122/123)
    const uint8_t DEAD_ZONE = 20;  // Adjust as needed
    
    uint8_t joyX, joyY;
    
    for(;;) {
        // It’s a good idea to make local copies in case the IRQ updates rx_data.
        joyX = rx_data[0];
        joyY = rx_data[1];
        
        // Check forward/backward first (X axis)
        if (joyX > (X_NEUTRAL + DEAD_ZONE)) {
            // If pushed significantly to the right on the X axis, move forward.
            forwardMotor();
        } else if (joyX < (X_NEUTRAL - DEAD_ZONE)) {
            // If pushed significantly to the left on the X axis, reverse.
            reverseMotor();
        }
        // Otherwise, check left/right using the Y axis.
        else if (joyY > (Y_NEUTRAL + DEAD_ZONE)) {
            // If pushed significantly upward (or right depending on wiring), turn right.
            rightTurn();
        } else if (joyY < (Y_NEUTRAL - DEAD_ZONE)) {
            // If pushed significantly downward (or left), turn left.
            leftTurn();
        } else {
            // In the dead zone, stop the motors.
            stopMotor();
        }
        
        osDelay(10); // Short delay (10ms) to allow for a new reading
    }
}

// ####################### START OF LED TASK ###########################

// The numbering here looks weird but follows a line on the left-side of the board
#define RED_LED_1 8 		// 	PortB Pin 8 
#define GREEN_LED_1 7 //	PortC Pin 7
#define GREEN_LED_2 0 //	PortC Pin 0
#define GREEN_LED_3 3 //	PortC Pin 3
#define GREEN_LED_4 4 //	PortC Pin 4
#define GREEN_LED_5 5 //	PortC Pin 5
#define GREEN_LED_6 6 //	PortC Pin 6
#define GREEN_LED_7 10 //	PortC Pin 10
#define GREEN_LED_8 11 //	PortC Pin 11
#define GREEN_LED_9 12 //	PortC Pin 12
#define GREEN_LED_10 13//	PortC Pin 13
#define MASK(x) (1 << (x))


void InitLEDGPIO(void) {
	
	// Green LED
  //Enable CLock to PORTC
  SIM->SCGC5 |= (SIM_SCGC5_PORTC_MASK); 
  
  //Configure MUX settings 
  PORTC->PCR[GREEN_LED_1] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTC->PCR[GREEN_LED_1] |= PORT_PCR_MUX(1); //Assign output
  
  PORTC->PCR[GREEN_LED_2] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTC->PCR[GREEN_LED_2] |= PORT_PCR_MUX(1); //Assign output 
  
  PORTC->PCR[GREEN_LED_3] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTC->PCR[GREEN_LED_3] |= PORT_PCR_MUX(1); //Assign output 
   
  PORTC->PCR[GREEN_LED_4] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTC->PCR[GREEN_LED_4] |= PORT_PCR_MUX(1); //Assign output 
   
  PORTC->PCR[GREEN_LED_5] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTC->PCR[GREEN_LED_5] |= PORT_PCR_MUX(1); //Assign output 
   
  PORTC->PCR[GREEN_LED_6] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTC->PCR[GREEN_LED_6] |= PORT_PCR_MUX(1); //Assign output 
   
  PORTC->PCR[GREEN_LED_7] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTC->PCR[GREEN_LED_7] |= PORT_PCR_MUX(1); //Assign output 
   
  PORTC->PCR[GREEN_LED_8] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTC->PCR[GREEN_LED_8] |= PORT_PCR_MUX(1); //Assign output 
  
  PORTC->PCR[GREEN_LED_9] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTC->PCR[GREEN_LED_9] |= PORT_PCR_MUX(1); //Assign output 
   
  PORTC->PCR[GREEN_LED_10] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTC->PCR[GREEN_LED_10] |= PORT_PCR_MUX(1); //Assign output 
  
  //Set Data Direction Registers for PortB and PortD
  PTC->PDDR |= (MASK(GREEN_LED_1) | MASK(GREEN_LED_2) | MASK(GREEN_LED_3) | MASK(GREEN_LED_4) | MASK(GREEN_LED_5) | 
                MASK(GREEN_LED_6) | MASK(GREEN_LED_7) | MASK(GREEN_LED_8) | MASK(GREEN_LED_9) | MASK(GREEN_LED_10));
								
								
								
								
	// RED LED 
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    
	PORTB->PCR[RED_LED_1] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED_1] |= PORT_PCR_MUX(1);
	
	PTB->PDDR |= MASK(RED_LED_1);
};




void led_toggler(int colour_current) { //, int colour_previou
  PTC->PCOR = (MASK(GREEN_LED_1)|MASK(GREEN_LED_2)|MASK(GREEN_LED_3)|MASK(GREEN_LED_4)|MASK(GREEN_LED_5)|MASK(GREEN_LED_6)|MASK(GREEN_LED_7)|MASK(GREEN_LED_8)|MASK(GREEN_LED_9)|MASK(GREEN_LED_10));

  PTC->PSOR = MASK(colour_current);
	osDelay(40); // TODO: Fine-Tune this. I converted 0xf0f0 directly to decimal
}


int greenPins[] = {GREEN_LED_1, GREEN_LED_2, GREEN_LED_3, GREEN_LED_4, GREEN_LED_5, GREEN_LED_6, GREEN_LED_7,GREEN_LED_8,GREEN_LED_9,GREEN_LED_10};

void green_led_left_to_right(){

    for (int i = 0; i < 10; i++){
      led_toggler(greenPins[i]);
//Can add delay here as required
    }
  
}



void green_led_remain() { //, int colour_previou
  PTC->PSOR = (MASK(GREEN_LED_1)|MASK(GREEN_LED_2)|MASK(GREEN_LED_3)|MASK(GREEN_LED_4)|MASK(GREEN_LED_5)|MASK(GREEN_LED_6)|MASK(GREEN_LED_7)|MASK(GREEN_LED_8)|MASK(GREEN_LED_9)|MASK(GREEN_LED_10));
  //delay();
}



void toggleRedLED500ms (){    // Red LEDs go on for 500ms and off for 500ms
    
    PTB->PCOR |= MASK(RED_LED_1);
    osDelay(500);
    PTB->PSOR |= MASK(RED_LED_1);
    osDelay(500);
    
}
	
void toggleREDLED250ms (){    // Red LEDs go on for 250ms and off for 250ms
    
    PTB->PCOR |= MASK(RED_LED_1);
    osDelay(250);
    PTB->PSOR |= MASK(RED_LED_1);
    osDelay(250);
    
}






// ####################### END OF LED TASK ###############################



void led_green_thread(void *argument)
{
	//uint8_t command = NODATA;
	uint8_t command = rx_IRQ_data; // this is horrible practice
	
	for(;;)
	{
		//osMessageQueueGet(tGreenMsg, &command, NULL, 0);
		
		if (rx_IRQ_data == R2 || rx_IRQ_data == L2 || rx_IRQ_data == RJOYSTICK_RIGHT || rx_IRQ_data == RJOYSTICK_LEFT || rx_IRQ_data == LJOYSTICK_UP || rx_IRQ_data == LJOYSTICK_DOWN) 
		{
			green_led_left_to_right();
		}
		else { // every other command
			green_led_remain();
		}
	}
	
}

void led_red_thread(void *argument)
{
	
	//uint8_t command = NODATA;
	
	uint8_t command = rx_IRQ_data; // this is horrible practice
	 
	for(;;)
	{
		//osMessageQueueGet(tRedMsg, &command, NULL, 0);
		
		if (rx_IRQ_data == R2 || rx_IRQ_data == L2 || rx_IRQ_data == RJOYSTICK_RIGHT || rx_IRQ_data == RJOYSTICK_LEFT || rx_IRQ_data == LJOYSTICK_UP || rx_IRQ_data == LJOYSTICK_DOWN) 
		{
			toggleRedLED500ms(); // TODO: CHECK IF THIS IS CORRECT 250MS
		}
		else { // every other command --> stationary
			toggleREDLED250ms();
		}
	}
}



#define R2 0x01
#define L2 0x02
#define SQUARE 0x03
#define TRIANGLE 0x04
#define CROSS 0x05
#define CIRCLE 0x06
#define RJOYSTICK_RIGHT 0x07
#define RJOYSTICK_LEFT 0x08
#define LJOYSTICK_UP 0x10
#define LJOYSTICK_DOWN 0x09



#define NOTE_C4  261
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define REST 0

#define MOD(x) (37500/x)

void delay_ms(uint32_t delay)
{
    SysTick->LOAD = (SystemCoreClock / 1000) - 1; // Count down from this value
    SysTick->VAL = 0; //Clear current value register
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; //Enable SysTick

    for(uint32_t i = 0; i < delay; i++) {
        //Wait until COUNTFLAG is set
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
    }

    SysTick->CTRL = 0; //Disable SysTick
}

#define PTB0_Pin 0
#define PTB1_Pin 1
#define PTE30_Pin 30 //TPM0_CH3
			

void initAudioPWM(void) {
	
	//Enables the clock gate for Port E
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	
	PORTE->PCR[PTE30_Pin] &= ~PORT_PCR_MUX_MASK; //Clear bit 10 to 8
	PORTE->PCR[PTE30_Pin] |= PORT_PCR_MUX(3); //Enable Timer function of pin
	
	//Enables clock gate for TPM1, page 208
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK; 
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK; //Clear bit 25 to 24, datasheet page 195
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); //Set 01 to bit 25 to 24, MCGFLLCLK clock or MCGPLLCLK/2 is used as clock source for TPM counter clock
	
	TPM0->MOD = 7500; //Set Modulo value = 4 800 000 / 128 = 375 000 / 7500 = 50 Hz
	
	//Datasheet 553, LPTPM means low power timer/pulse width modulator module
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); //Clears bit 4 to 0, 2 to 0 for PS, 4 to 3 for CMOD
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7)); // LPTPM counter clock mode is selected as 01 (LPTPM counter increments on every LPTPM counter clock), Prescaler 128 
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK); //Clears CPWMS (Centre-aligned PWM select). Aka mode = 0 which means LPTPM counts up
	
	//Datasheet 555
	TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); //Clears bit 5 to 2, disabling channel mode 
	TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // CPWMS = 0, ELSnB:ELSnA = 0b10, MSnB:MSnA = 0b10, this means Mode = Edge-aligned PWM, Config = High-true pulses (clear Output on match, setOutput on reload)
}

int twinkle[] = {
    NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_G4, REST,
    NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_C4, REST,
    NOTE_G4, NOTE_G4, NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, REST,
    NOTE_G4, NOTE_G4, NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, REST,
    NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_G4, REST,
    NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_C4, REST
};

int finish_run[] = {
    NOTE_C4, NOTE_E4, NOTE_G4, NOTE_C5, REST
};

#define MOD_music(x) (75000/x)
void play_twinkle()
{
	int notes_num = sizeof(twinkle)/ sizeof(twinkle[0]);
	int beats_per_min = 120;
	
	int one_beat = 60000 / beats_per_min; //60000 ms = 60 seconds
	
	for(int i = 0; i < notes_num; i++)
	{
		int curr_musical_note = twinkle[i];
		
		int period = MOD_music(curr_musical_note);
		
		TPM0->MOD = period;
		TPM0_C3V = period / 6; 
		
		delay_ms(one_beat); //all equal in length
	}
};

void play_finish(){
    int notes_num = sizeof(finish_run)/ sizeof(finish_run[0]);
	int beats_per_min = 120;
	
	int one_beat = 60000 / beats_per_min; //60000 ms = 60 seconds
	
	for(int i = 0; i < notes_num; i++)
	{
		int curr_musical_note = finish_run[i];
		
		int period = MOD_music(curr_musical_note);
		
		TPM0->MOD = period;
		TPM0_C3V = period / 6; 
		
		delay_ms(one_beat); //all equal in length
	}
}

void delay_ms(uint32_t delay)
{
    SysTick->LOAD = (SystemCoreClock / 1000) - 1; // Count down from this value
    SysTick->VAL = 0; // Clear current value register
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick

    for(uint32_t i = 0; i < delay; i++) {
        // Wait until the COUNTFLAG is set
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
    }

    SysTick->CTRL = 0; // Disable SysTick
}

#define PTB0_Pin 0
#define PTB1_Pin 1
#define PTE30_Pin 30 // TPM0_CH3
			

void audio_thread(void *argument) {	
	
	for(;;)
	{
        //osMessageQueueGet(tAudioMsg, &command, NULL, 0);
        
        if (rx_IRQ_data == TRIANGLE) {
            play_twinkle();
        }
        else {
            play_finish();
        }
		
	}
}


// ####################### END OF AUDIO TASK ###############################



void app_main (void *argument) {
 
  // ...
  for (;;) {
		;
	
	
	}
}

int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	 
	// MOTOR 
  initUART2(BAUD_RATE);
  initMotorPWM();
  offAllLed();
  stopMotor();
	
	// LED
	InitLEDGPIO();
	
	// AUDIO
	initAudioPWM();
	
 
  osKernelInitialize();        
	
	osThreadNew(motor_thread, NULL, NULL);
	osThreadNew(led_green_thread, NULL, NULL);
	osThreadNew(led_red_thread, NULL, NULL);
	osThreadNew(audio_thread,NULL,NULL);

  //osThreadNew(app_main, NULL, NULL);    // Create application main thread
	
	
	
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
