#include "msp.h"
#include <msp.h>
#include "set_DCO.h"
#include <msp432p401r.h>
#include <stdio.h>
#include <stdint.h>

#define GAIN BIT5
#define SHDN BIT4
#define CS BIT5
#define twoVolts 2482
#define ON 1
#define OFF 0

int state = ON;

// data =2482 for




/**
 * main.c
 */
void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	set_DCO(FREQ_12_MHz); //Set SMCLK to 12 MHz

	EUSCI_B0->CTLW0 |= EUSCI_BCTL0_SWRST; // Reset serial peripheral
	EUSCI_B0->CTLW0 |= EUSCI_B_CKPL       //Clock inactive for high mode
	                 | EUSCI_B_MSB        //MSB first
                     | EUSCI_B_MST        //Master mode for SPI
                     | EUSCI_B_SYNC       //SPI is synchronous
                     | EUSCI_B_UCSSEL_2   //Select SMCLK
                     | EUSCI_B_SWRST;     //Keep peripheral reset
	EUSCI_B0->BRW = 0x01;                 //Divide to keep
                                          //SMCLK at desired speed
    EUSCI_B0->CTLW0 &= ~(EUSCI_B_CTL0_SWRST);  //Activate SPI Peripheral


	CS->KEY = CS_KEY_VAL;
	CS->CTL0 = CS_CTL0_DCORSEL_0;          //1.5 MHz default
	CS->KEY = 0;


   // interrupt stuff
	TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE; // TACCR0 interrupt enabled
	TIMER_A0->CCR[0] = 15122;
	TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | // SMCLK, up mode
	                TIMER_A_CTL_MC__UP;

    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;    // Enable sleep on exit from ISR


	P1->SEL0 |= BIT5|BIT6;                     //Select EUSCI_B0
    P1->SEL1 &= ~(BIT5|BIT6);                  //for SMCLK and SIMO
    P6->DIR |= CS;                           //Set Port 6.5 as output for CS

    __enable_irq();

    NVIC->ISER[0] = 1 << ((TA0_0_IRQn) & 31);

    while(1){
        __sleep_();
        }
    }

void TA0_0_IRQHandler(void) {
    uint16_t data;
    uint8_t hiByte, lowByte;
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;

    if (state == ON) {
        data = 2482;
        state = OFF;
    }
    else {
        data = 0;
        state = ON;
    }

    lowByte = data & 0xFF;        //Mask byte
    hiByte = (data>>8) & 0x0F;    //Shift upper 4 bits
    hiByte |= GAIN|SHDN;          //GAIN=BIT5, SHDN=BIT4

    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TxIFG));
                                             //Wait for TxBUF empty
    EUSCI_B0->TXBUF = hiByte;

    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RxIFG));
                                             //Wait for receive buffer to know
                                             //when done transitioning
    P6->OUT ^= BIT5;              //Set CS high
    delay_us(200);                //Wait to change voltage

    }
}