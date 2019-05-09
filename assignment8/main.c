#include "msp.h"
#include "dac.h"
#include "math.h"
#include "delay.h"
#include "set_DCO.h"
#include <string.h>
#include <stdlib.h>

/**
 * main.c
 */


void print_char(char letter);
void print_string(char* string);

int flag = 0;
char inValue[5];
int index = 0;

int convertToDecimal(char* val)
{
    int i, num = 0;
    //Find the decimal representation of
    for(i=0; i<4 && val[i] != '\0'; i++)
    {
        if(val[i]>='0' && val[i]<='9')
        {
            num *=10;
            num += val[i] - 48;
        }
    }
    if(num > 4095)
        return 4095;
    return num;
}

void main(void)
{
    int voltage = 0;

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;  // stop watchdog timer

    set_DCO(FREQ_3_MHz);


	//DAC init SPI
	P1->SEL0 |= BIT5|BIT6;                     //Select EUSCI_B0
	P1->SEL1 &= ~(BIT5|BIT6);                  //for SMCLK and SIMO
	P6->DIR |= EUSCI_CS;                           //Set Port 6.5 as output for CS
	P6->OUT |= EUSCI_CS;
	EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SWRST; // Reset serial peripheral
	    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_CKPL       //Clock inactive for high mode
	                     | EUSCI_B_CTLW0_MSB        //MSB first
	                     | EUSCI_B_CTLW0_MST        //Master mode for SPI
	                     | EUSCI_B_CTLW0_SYNC       //SPI is synchronous
	                     | EUSCI_B_CTLW0_UCSSEL_2   //Select SMCLK
	                     | EUSCI_B_CTLW0_SWRST;     //Keep peripheral reset
	    EUSCI_B0->BRW = 0x01;                 //Divide to keep SMCLK at desired speed
	    EUSCI_B0->CTLW0 &= ~(EUSCI_B_CTLW0_SWRST);  //Activate SPI Peripheral


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
	__enable_irq(); //enable global interrupts
	//EUSCI_A0->TXBUF = 0x31;


	// TODO: remove
	print_char('h');
	print_char('e');
	print_char('l');
	print_char('l');


	while(1) {
	    if (flag)
	    {
	        print_string(inValue);
	        voltage = convertToDecimal(inValue);
	        if (voltage < 4096 && voltage >= 0)
	            driveDAC(voltage); //set_voltage(voltage);
	        flag = 0;
	        index = 0;
	        voltage = 0;
	    }
	}
}

void print_char(char letter)
{
    while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));    //wait for TX buffer to empty
    EUSCI_A0->TXBUF = letter;   // transmit character
}

void print_string(char* string)
{
    int i;

    for (i = 0; i < 4 && string[i] != '\0'; i++) {
        print_char(string[i]);
    }

//    print_char('\r');//0x1B);
}

void EUSCIA0_IRQHandler(void)
{
    char letter;
    if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG) {
        // get user input
	    letter = EUSCI_A0->RXBUF;   // transmit character

	    // echo user input
	    while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));    //wait for TX buffer to empty
	    EUSCI_A0->TXBUF = letter;   // transmit character

        // parse user input
	    if (letter >= '0' && letter <= '9' && index < 4)
	    {
	        inValue[index] = letter;
            index++;
	    }

	    if (letter == 0x0D)
	    {
            inValue[index] = '\0';
            flag = 1;
	    }
	}
}
