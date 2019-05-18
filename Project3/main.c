#include <math.h>
#include "msp.h"
#include "set_DCO.h"
#include <stdint.h>
/**
 * main.c
 */

void initUART()
{
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; //get into reset state

    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_PEN //parity odd
            | EUSCI_A_CTLW0_SPB // 2 stop bits
            | EUSCI_A_CTLW0_MODE_0 // UART mode
            | EUSCI_A_CTLW0_UCSSEL_2 //SMCLK
            | EUSCI_A_CTLW0_SWRST; //keep in reset

    EUSCI_A0->BRW = 0x01; // UCBRx calculation

    EUSCI_A0->MCTLW = (10<<EUSCI_A_MCTLW_BRF_OFS)
                | EUSCI_A_MCTLW_OS16; //0S16 = 1

    P1->SEL0 |= (BIT2|BIT3); //select EUSCI_A0
    P1->SEL1 &= ~(BIT2|BIT3);

    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; //take out of reset mode

    EUSCI_A0->IE |= EUSCI_A_IE_RXIE; //enable USCI_A0 RX interrupts

    NVIC->ISER[0] = 1<<(EUSCIA0_IRQn & 31);
}

void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	set_DCO(FREQ_48_MHz);                            //set DCO to 48 MHz

	// set SMCLK to 48MHz
	CS->KEY = CS_KEY_VAL; // unlock CS registers
	CS->CTL1 |= CS_CTL1_DIVS_0 | CS_CTL1_SELS_3; //set SELS to select DCO source for SMCLK
	                                             //set DIVS to divide by 1
	CS->KEY = 0; // lock the CS registers

	initUART();

    __enable_irq(); //enable global interrupts

}

void EUSCIA0_IRQHandler(void)
{
    char letter;

    if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG)
    {
        // get user input
        letter = EUSCI_A0->RXBUF;   //receive character

        // echo user input
        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));    //wait for TX buffer to empty
        EUSCI_A0->TXBUF = letter;   // transmit character
    }
}
