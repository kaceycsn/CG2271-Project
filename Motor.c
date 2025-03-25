#include "MKL25Z4.h"

// Motor control pin definitions (renamed for clarity)
#define L_BACK_FW      0   // PTB0  - TPM1_CH0
#define L_BACK_BW      1   // PTB1  - TPM1_CH1
#define R_BACK_FW      2   // PTB2  - TPM2_CH0
#define R_BACK_BW      3   // PTB3  - TPM2_CH1
#define L_FRONT_FW     10  // PTB10 - TPM0_CH1
#define L_FRONT_BW     11  // PTB11 - TPM0_CH0
#define R_FRONT_FW     12  // PTB12 - TPM1_CH0 (Alt location)
#define R_FRONT_BW     13  // PTB13 - TPM1_CH1 (Alt location)

#define MASK(x) (1 << (x))
#define DUTY_CYCLE 7500

volatile int speed = 2;

void initPWM(void) {
    // Enable clock gating for Port B
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
    
    // Set pins to TPM alternate function (ALT 3)
    PORTB->PCR[L_BACK_FW]     = PORT_PCR_MUX(3);
    PORTB->PCR[L_BACK_BW]     = PORT_PCR_MUX(3);
    PORTB->PCR[R_BACK_FW]     = PORT_PCR_MUX(3);
    PORTB->PCR[R_BACK_BW]     = PORT_PCR_MUX(3);
    PORTB->PCR[L_FRONT_FW]    = PORT_PCR_MUX(3);
    PORTB->PCR[L_FRONT_BW]    = PORT_PCR_MUX(3);
    PORTB->PCR[R_FRONT_FW]    = PORT_PCR_MUX(3);
    PORTB->PCR[R_FRONT_BW]    = PORT_PCR_MUX(3);

    // Enable TPM0, TPM1, TPM2
    SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK | SIM_SCGC6_TPM1_MASK | SIM_SCGC6_TPM2_MASK;

    // Set TPM clock source
    SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

    // Set MOD value for all TPMs
    TPM0->MOD = DUTY_CYCLE;
    TPM1->MOD = DUTY_CYCLE;
    TPM2->MOD = DUTY_CYCLE;

    // Configure TPMs for edge-aligned PWM, prescaler 128
    TPM0->SC = TPM_SC_CMOD(1) | TPM_SC_PS(7);
    TPM1->SC = TPM_SC_CMOD(1) | TPM_SC_PS(7);
    TPM2->SC = TPM_SC_CMOD(1) | TPM_SC_PS(7);

    TPM0->SC &= ~TPM_SC_CPWMS_MASK;
    TPM1->SC &= ~TPM_SC_CPWMS_MASK;
    TPM2->SC &= ~TPM_SC_CPWMS_MASK;

    // Enable PWM on each TPM channel
    TPM1_C0SC = TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1); // L_BACK_FW
    TPM1_C1SC = TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1); // L_BACK_BW
    TPM2_C0SC = TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1); // R_BACK_FW
    TPM2_C1SC = TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1); // R_BACK_BW
    TPM0_C1SC = TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1); // L_FRONT_FW
    TPM0_C0SC = TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1); // L_FRONT_BW
    TPM1_C0SC = TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1); // R_FRONT_FW
    TPM1_C1SC = TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1); // R_FRONT_BW
}

void stopAllMotors() {
    TPM0_C0V = TPM0_C1V = 0;
    TPM1_C0V = TPM1_C1V = 0;
    TPM2_C0V = TPM2_C1V = 0;
}

void forwardAll() {
    TPM0_C1V = DUTY_CYCLE; // L_FRONT_FW
    TPM1_C0V = DUTY_CYCLE; // L_BACK_FW
    TPM2_C0V = DUTY_CYCLE; // R_BACK_FW
    TPM1_C0V = DUTY_CYCLE; // R_FRONT_FW
}

void reverseAll() {
    TPM0_C0V = DUTY_CYCLE; // L_FRONT_BW
    TPM1_C1V = DUTY_CYCLE; // L_BACK_BW
    TPM2_C1V = DUTY_CYCLE; // R_BACK_BW
    TPM1_C1V = DUTY_CYCLE; // R_FRONT_BW
}

void turnLeft() {
    TPM0_C0V = DUTY_CYCLE / speed; // L_FRONT_BW
    TPM1_C1V = DUTY_CYCLE / speed; // L_BACK_BW
    TPM2_C0V = DUTY_CYCLE / speed; // R_BACK_FW
    TPM1_C0V = DUTY_CYCLE / speed; // R_FRONT_FW
}

void turnRight() {
    TPM0_C1V = DUTY_CYCLE / speed; // L_FRONT_FW
    TPM1_C0V = DUTY_CYCLE / speed; // L_BACK_FW
    TPM2_C1V = DUTY_CYCLE / speed; // R_BACK_BW
    TPM1_C1V = DUTY_CYCLE / speed; // R_FRONT_BW
}


int main (void) {
    initPWM();
    stopMotor();
    
    while(1) {
        forwardMotor();
        delay(0x80000);
        stopMotor();
        reverseMotor();
        delay(0x80000);
        stopMotor();
        rightTurn();
        delay(0x80000);
        leftTurn();
        stopMotor();
        delay(0x80000);
    }
}
