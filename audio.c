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

