#include "msp.h"
#include "set_DCO.h"
#include <msp432p401r.h>
#include <stdio.h>
#include <stdint.h>
#include "delay.h"
#include "dac.h"
int state = ON;
volatile int count = 0;
int data = 1441;

/**
 * main.c
 */

void TA0_0_IRQHandler(void) {
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG; //clear the interrupt flag
    if (state == ON) {
        data++;
        if (data == 3573)
            state = OFF;
    }
    else {
        data--;
        if (data == 1441)
            state = ON;
    }
    driveDAC(data);
}

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    set_DCO(FREQ_48_MHz); //Set SMCLK to 12 MHz

    //set SMCLK
    // change DC0 from default of 3MHz to 12MHz.
    CS->KEY = CS_KEY_VAL; // unlock CS registers
    CS->CTL1 |= CS_CTL1_DIVS_2 | CS_CTL1_SELS_3; //set SELS and DIVS
    CS->KEY = 0; // lock the CS registers


    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SWRST; // Reset serial peripheral
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_CKPL       //Clock inactive for high mode
                     | EUSCI_B_CTLW0_MSB        //MSB first
                     | EUSCI_B_CTLW0_MST        //Master mode for SPI
                     | EUSCI_B_CTLW0_SYNC       //SPI is synchronous
                     | EUSCI_B_CTLW0_UCSSEL_2   //Select SMCLK
                     | EUSCI_B_CTLW0_SWRST;     //Keep peripheral reset
    EUSCI_B0->BRW = 0x01;                 //Divide to keep SMCLK at desired speed

    // interrupt stuff
    TIMER_A0->CCTL[0] |= TIMER_A_CCTLN_CCIE; // TACCR0 interrupt enabled
    TIMER_A0->CCR[0] = 48;
    TIMER_A0->CTL |= TIMER_A_CTL_SSEL__SMCLK | // SMCLK, up mode
                    TIMER_A_CTL_MC__UP;

    //SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;    // Enable sleep on exit from ISR

    P1->SEL0 |= BIT5|BIT6;                     //Select EUSCI_B0
    P1->SEL1 &= ~(BIT5|BIT6);                  //for SMCLK and SIMO
    P6->DIR |= EUSCI_CS;                           //Set Port 6.5 as output for CS
    EUSCI_B0->CTLW0 &= ~(EUSCI_B_CTLW0_SWRST);  //Activate SPI Peripheral

    __enable_irq();
    NVIC->ISER[0] = 1 << ((TA0_0_IRQn) & 31);
    while(1){

    }

}
