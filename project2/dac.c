/*
 * dac.c
 *
 *  Created on: May 1, 2019
 *      Author: tyfarris
 */
#include "dac.h"

void driveDAC(int data)

{
    uint8_t hiByte, lowByte;
    lowByte = data & 0xFF;        //Mask byte
    hiByte = (data>>8) & 0x0F;    //Shift upper 4 bits
    hiByte |= GAIN|SHDN;          //GAIN=BIT5, SHDN=BIT4
    P6->OUT &= ~EUSCI_CS; //set CS LOW
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG));
    //Wait for TxBUF empty
    EUSCI_B0->TXBUF = hiByte;
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG));
    //Wait for TxBUF empty
    EUSCI_B0->TXBUF = lowByte;
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG));
    //Wait for receive buffer to know
    //when done transitioning
    EUSCI_B0->RXBUF; //clears RX interrupt flag
    while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG));
    //Wait for receive buffer to know
    //when done transitioning
    P6->OUT |= BIT5;              //Set CS high

}

