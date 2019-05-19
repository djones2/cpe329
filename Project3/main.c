#include <math.h>
#include "msp.h"
#include "set_DCO.h"
#include <stdint.h>
/**
 * main.c
 */

enum State {DC = 0, AC = 1};

typedef struct Measurements
{
    char state;
    int freq;


}Measurements;


Measurements data;
int countTimer = 0;
int countEdges = 0;

void printChar(char letter)
{
    EUSCI_A0->TXBUF = letter;   // transmit character

    // echo user input
    while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));    //wait for TX buffer to empty
}

void printStr(char* str)
{
    int i = 0;

    while (str[i] != '\0')
    {
        printChar(str[i]);
        i++;
    }
}

void convertDecToAscii(int value)
{
    char str[8];
    int num = 100 * value; //to get rid of the decimals

    str[0] = (num / 100000) + '0';    //set the 1000's place
    str[1] = (num / 10000) % 10 + '0';     //set the 100's place
    str[2] = (num / 1000) % 10 + '0';      //set the 10's place
    str[3] = (num / 100) % 10 + '0';       //set the 1's place
    str[4] = '.';
    str[5] = (num / 10) % 10 + '0';       //set the 0.1's place
    str[6] = (num % 10) + '0';       //set the 0.01's place
    str[7] = '\0';

    printStr(str);
    printChar('\n');            //creates a new line
    printChar('\r');            //acts like enter
}

void initUART(void)
{
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; //get into reset state

    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_PEN //parity odd
            | EUSCI_A_CTLW0_SPB // 2 stop bits
            | EUSCI_A_CTLW0_MODE_0 // UART mode
            | EUSCI_A_CTLW0_UCSSEL_2 //SMCLK
            | EUSCI_A_CTLW0_SWRST; //keep in reset

    EUSCI_A0->BRW = 0x1A; // UCBRx calculation

    EUSCI_A0->MCTLW = (1 << EUSCI_A_MCTLW_BRS_OFS)  //set UCSRx to 1
                | (1 << EUSCI_A_MCTLW_BRF_OFS) //set UCBRFx to 1
                | EUSCI_A_MCTLW_OS16; //set 0S16 = 1

    P1->SEL0 |= (BIT2|BIT3); //select EUSCI_A0
    P1->SEL1 &= ~(BIT2|BIT3);

    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; //take out of reset mode

    EUSCI_A0->IE |= EUSCI_A_IE_RXIE; //enable USCI_A0 RX interrupts

    NVIC->ISER[0] |= 1 <<(EUSCIA0_IRQn & 31);
}

void EUSCIA0_IRQHandler(void)
{
    char letter;

    if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG)
    {
        EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;    //clear the interrupt flag

        // get user input
        letter = EUSCI_A0->RXBUF;   //receive character

        //update if state is AC or DC
        if (letter == 'a')
        {
            data.state = AC;
        }
        else if (letter == 'd')
        {
            data.state = DC;
        }

        printChar(letter);
    }
}


void TA0_0_IRQHandler(void){
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;      //clears the interrupt flag

    if (countTimer == 50)
    {
        P3->IE &= ~BIT0; //enable GPIO interrupts
        data.freq = countEdges;
        countEdges = 0;
        countTimer = 0;
    }
    else
    {
        countTimer++;
    }
}

void PORT3_IRQHandler()
{
    P3->IFG &= ~BIT0; //clear flag
    countEdges ++;
}


void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    set_DCO(FREQ_48_MHz);                            //set DCO to 48 MHz

    // set SMCLK to 48MHz and set ACLK to 128kHz
    CS->KEY = CS_KEY_VAL; // unlock CS registers
    CS->CTL1 |= CS_CTL1_DIVS_0 //set SELS to select DCO source for SMCLK
                | CS_CTL1_SELS_3   //set DIVS to divide by 1
                | CS_CTL1_SELS__REFOCLK;     //sets the ACLK source to REFO

    CS->CLKEN |= CS_CLKEN_REFOFSEL;               //sets REFO to 128kHz
    CS->KEY = 0; // lock the CS registers

    initUART();

    /*Sets the timerA0 to be in up mode*/
    TIMER_A0->CTL |= (TIMER_A_CTL_TASSEL_2|TIMER_A_CTL_MC_1 | TIMER_A_CTL_ID_3); // Use SMCLK in up mode and divide by 8
    TIMER_A0->EX0 |= TIMER_A_EX0_TAIDEX_1; //clock divide by 2 --> TIMER A0 runs on 3 MHz clock

     TIMER_A0->CCR[0] = 59130; //59500     // Period = 1 sec

     TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE;      // Enable interrupts on TIMER_A0

     NVIC->ISER[0] = 1 << (TA0_0_IRQn & 31);      // Enable CCR0 ISR

     P3->DIR &= ~BIT0; //P3.0 used for taking in waveform from comparator and calculating frequency
     P3->IES |= BIT0; //set interrupt on high to low transition
     P3->IFG &= ~BIT0; //clear flag
     P3->IE |= BIT0; //enable GPIO interrupts

     NVIC->ISER[1] = 1 << (PORT3_IRQn & 31);

     __enable_irq(); //enable global interrupts

    while(1)
    {
        //sample
        P3->IE |= BIT0; //enable GPIO interrupts
        //print
        convertDecToAscii(data.freq);

    }
}
