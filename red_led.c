#include "MKL25Z4.h"
#include "RTE_Components.h"
#include CMSIS_device_header
#include "cmsis_os2.h"

#define RED_LED 0 // PTD0
#define MASK(x) (1 << (x))

volatile int robot_moving = 1; // 1 = moving, 0 = stationary

// Initialize PTD0 as GPIO output for red LED
void initRedLED() {
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK; // Enable clock for Port D

    PORTD->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
    PORTD->PCR[RED_LED] |= PORT_PCR_MUX(1); // Set as GPIO

    PTD->PDDR |= MASK(RED_LED); // Set PTD0 as output
}

// Red LED controller thread: handles flashing based on movement state
void redLEDController(void *arg) {
    while (1) {
        int delay_time = robot_moving ? 500 : 250; // 500ms if moving, 250ms if stationary

        PTD->PCOR = MASK(RED_LED); // Turn ON 
        osDelay(delay_time);

        PTD->PSOR = MASK(RED_LED); // Turn OFF
        osDelay(delay_time);
    }
}

int main(void) {
    SystemCoreClockUpdate();
    initRedLED();

    osKernelInitialize();
    osThreadNew(redLEDController, NULL, NULL); // Start LED flashing thread
    osKernelStart();

    for (;;) {} // Idle loop
}
