#include "msp.h"
#include <msp.h>
#include "set_DCO.h"
#include <msp432p401r.h>
#include <stdio.h>
#include <stdint.h>



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
	EUSCI_B0->BRW = 0x01;                 //Divide (by 1) to keep
                                          //SMCLK at full speed
	P1->SEL0 |= BIT5|BIT6;                //Select EUSCI_B0
    P1->SEL1 &= (~BIT5|BIT6);             //for SMCLK and SIMO
    P6->DIR |= BIT5;                      //Set Port 6.5 as output for CS
    EUSCI_B0->CTLW0 &= ~(EUSCI_B_CTL0_SWRST);
                                          //Activate SPI Peripheral
    uint16_t data;
    uint8_t hiByte, lowByte;
    while(1){

        P6->OUT |= BIT5;                  //Set CS high

        for (data=0; data<4096; data++){
            lowByte = data & 0xFF;        //Mask byte
            hiByte = (data>>8) & 0x0F;    //Shift upper 4 bits
            hiByte |= GAIN|SHDN;          //GAIN=BIT5, SHDN=BIT4???
            P6->OUT &= ~BIT5;             //Set CS low

            while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TxIFG));
                                          //Wait for TxBUF empty
            EUSCI_B0->TXBUF = hiByte;

            while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RxIFG));
                                          //Wait for receive buffer to know
                                          //when done transitioning
            P6->OUT |= BIT5;              //Set CS high
            delay_us(200;)                //Wait to change voltage

            //Can add additional flag checks for safety
        }
    }
}
