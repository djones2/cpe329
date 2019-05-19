#include <math.h>
#include "msp.h"
#include "set_DCO.h"
#include <stdint.h>
#include <string.h>
#include "delay.h"
/**
 * main.c
 */

enum State {DC = 0, AC = 1};

char state;
volatile int freq = 0;
volatile int min;
volatile int max;
volatile int dc_voltage;
volatile int pk_pk;
int countTimer = 0;
int countEdges = 0;

void printChar(char letter)
{
    // echo user input
    while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));    //wait for TX buffer to empty

    EUSCI_A0->TXBUF = letter;   // transmit character
}

void printStr(char* str, int length)
{
    int i = 0;

    for(i=0; i<length; i++)
    {
        if(str[i] == '\\')
            printChar(27);
        else
            printChar(str[i]);
    }
}

void convertDecToAscii(float value)
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

    printStr(str, 7);
    printChar('\n');
    printChar('\r');
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
            state = AC;
        }
        else if (letter == 'd')
        {
            state = DC;
        }

        printChar(letter);
    }
}


void TA0_0_IRQHandler(void){
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;      //clears the interrupt flag

    if (countTimer == 50)
    {
        P3->IE &= ~BIT0; //disable interrupting for rising edges
        freq = countEdges;
        countEdges = 0;
        countTimer = 0;
        P3->IE |= BIT0; //disable interrupting for rising edges
    }
    else
    {
        countTimer++;
    }
}

void TA0_N_IRQHandler(void){
    if(TIMER_A0->CCTL[1] & TIMER_A_CCTLN_CCIFG)
    {
        TIMER_A0->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG;      //clears the interrupt flag
        max = -1;
        min = 16383;
        ADC14->CTL0 |= ADC14_CTL0_SC; //start conversion
    }
}

void PORT3_IRQHandler()
{
    P3->IFG &= ~BIT0; //clear flag
    countEdges ++;
}

void ADC14_IRQHandler()
{
    volatile uint16_t readValue;
    readValue = ADC14->MEM[0]; //read ADC value (clears interrupt when reading value
    if(readValue > max)
        max = readValue;
    else if(readValue < min)
        min = readValue;
}

void sampleData()
{
    int fr;
    ADC14->IER0 = ADC14_IER0_IE0; //enable interrupts on mem[0]
    fr = 50 * freq;
    fr = 3000000/fr;
    TIMER_A0->CCR[1] = fr;
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_CCIE;      // Enable interrupts
    delay_us(2000000); // delay 1 second (max period)
    dc_voltage = (max + min)/2;
    pk_pk = (max - min);
}

void main(void)
{
    float convertedDC = 0;
    float convertedPk = 0;

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

     TIMER_A0->CCR[0] = 59130;      // Period = 1 sec
     TIMER_A0->CCR[1] = 1000; // Fix value in sampleData() function

     TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE;      // Enable interrupts on TIMER_A0
     //TIMER_A0->CCTL[1] = TIMER_A_CCTLN_CCIE;      // Enable interrupts

     NVIC->ISER[0] = 1 << (TA0_0_IRQn & 31);      // Enable CCR0 ISR
     NVIC->ISER[0] = 1 << (TA0_N_IRQn & 31);      // Enable CCR1 ISR


     P3->DIR &= ~BIT0; //P3.0 used for taking in waveform from comparator and calculating frequency
     P3->IES |= BIT0; //set interrupt on high to low transition
     P3->IFG &= ~BIT0; //clear flag
     P3->IE |= BIT0; //enable GPIO interrupts

     NVIC->ISER[1] = 1 << (PORT3_IRQn & 31);

     P5->SEL1 |= BIT4 | BIT5;                // Configure P5.4/5 for ADC
     P5->SEL0 |= BIT4 | BIT5;

     //configure ADC
     ADC14->CTL0 &= ~ADC14_CTL0_ENC; //disables ADC for configuration
     ADC14->CTL0 = ADC14_CTL0_SHP //sample pulse mode and use internal sample timer
             | ADC14_CTL0_SSEL_4 //SMCLK
             | ADC14_CTL0_SHT0_0 //sample 4 clocks
             | ADC14_CTL0_ON; // turn on ADC14
     ADC14->CTL1 = (0 << ADC14_CTL1_CSTARTADD_OFS) //start at mem address 0
             | ADC14_CTL1_RES_3; //14 bit resolution
     ADC14->MCTL[0] = ADC14_MCTLN_INCH_14; //select channel 14
     ADC14->IER0 = ADC14_IER0_IE0; //enable interrupts on mem[0]
     ADC14->CTL0 |= ADC14_CTL0_ENC; //enable ADC14
     NVIC->ISER[0] = 1 << (ADC14_IRQn & 31);

     __enable_irq(); //enable global interrupts

    while(1)
    {
        printStr("Freq: ", 6);
        convertDecToAscii((float)freq);

        sampleData();

        printStr("DC offset: ", 11);
        convertedDC = dc_voltage/4948;
        convertDecToAscii(convertedDC);

        printStr("Peak to Peak: ", 14);
        convertedPk = pk_pk/4948;
        convertDecToAscii(convertedPk);
    }
}
