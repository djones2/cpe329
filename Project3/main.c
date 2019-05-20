#include <math.h>
#include "msp.h"
#include "set_DCO.h"
#include <stdint.h>
#include <string.h>
#include "delay.h"
/**
 * main.c
 */

#define NUM_SAMPLES 50
#define DC_DIVIDER 5000.0
#define PK_DIVIDER 5050.0//4950.0
#define RMS_DIVIDER 5060.0
enum State {DC = 0, AC = 1};

int bool = 0;
char state;
volatile float vrms;
volatile int freq = 0;
volatile int min;
volatile int max;
volatile int dc_voltage;
volatile int pk_pk;
int countTimer = 0;
int countEdges = 0;
int array[50];
int index = 0;

float convertedDC;
float convertedPk;

void screen_init_ac();
void screen_init_DC();

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
    printStr("\e[2C", 4);
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
            screen_init_ac();

        }
        else if (letter == 'd')
        {
            state = DC;
            screen_init_DC();
        }

        //printChar(letter);
    }
}


void TA0_0_IRQHandler(void){
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;      //clears the interrupt flag

    if (countTimer == NUM_SAMPLES)   //time is 1 sec
    {
        //TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIE;      //disable interrupts for the freq calculations
        P3->IE &= ~BIT0; //disable interrupting for rising edges
        freq = countEdges;
        vrms = 0;
        max = 0;
        min = 16383;
        countEdges = 0;
        countTimer = 0;
        bool = 0;
    }
    else
    {
        countTimer++;
        P3->IE |= BIT0;
    }
}


void TA0_N_IRQHandler(void){
    if(TIMER_A0->CCTL[1] & TIMER_A_CCTLN_CCIFG)
    {
        TIMER_A0->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG;      //clears the interrupt flag
        ADC14->CTL0 |= ADC14_CTL0_SC; //start conversion
    }
}

void PORT3_IRQHandler()
{
    P3->IFG &= ~BIT0; //clear flag
    countEdges++;
}

void ADC14_IRQHandler()
{
    volatile uint16_t readValue;
    readValue = ADC14->MEM[0]; //read ADC value (clears interrupt when reading value
    array[index] = readValue;
    index ++;
    if(index == 50)
        index = 0; //should not ever get to this state
    //vrms += ((float)readValue * readValue)/(RMS_DIVIDER * RMS_DIVIDER);

    if(readValue > max)
    {
        max = readValue;
    }
    else if(readValue < min)
    {
        min = readValue;
    }
}

void sampleData()
{
    int i;
    int fr;
    bool = 1;
    TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE;
    //while(bool==1);
    _delay_cycles(96000000);
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIE;
    ADC14->IER0 = ADC14_IER0_IE0; //enable interrupts on mem[0]
    fr = NUM_SAMPLES * freq;
    fr = 3000000/fr;
    TIMER_A0->CCR[1] = fr;
    index = 0;
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_CCIE;      // Enable interrupts for ADC conversions
    _delay_cycles(96000000); // delay 1 second (max period)
    TIMER_A0->CCTL[1] &= ~TIMER_A_CCTLN_CCIE;
    vrms = 0;
    for(i=0; i<NUM_SAMPLES; i++)
        vrms += (((float)array[i]) * array[i])/(RMS_DIVIDER * RMS_DIVIDER);
}

void freq_graph(int freq){
    printStr("\e[K", 3);
    int i;
    freq = freq/25;
    for(i=0; i<freq; i++){
        printChar('*');
    }
}

void voltage_graph(float voltage){
    int i;
    //voltage = voltage*10;
    printStr("\e[K", 3);
    voltage = voltage*10;
    int j = (int)voltage;
    for(i=0; i<j; i++){
        printChar('*');
    }

}

void screen_init_DC(){
     printStr("\e[5m", 4); // blinking mode to see cursor
     printStr("\e[2J", 4); // clear
     printStr("\e[H", 4);  // set cursor home
     printStr("DC Voltage: ", 11);
     printStr("\e[E", 3);
     printStr("DC: ", 4);
     printStr("\e[E", 3);
     printStr("\e[5C", 4);
     printStr("0", 1);
     printStr("\e[8C", 5);
     printStr("1", 1);
     printStr("\e[9C", 5);
     printStr("2", 1);
     printStr("\e[9C", 5);
     printStr("3V", 2);
     printStr("\e[H", 4);
     printStr("\e[11C", 5);
}


void screen_init_ac(){
     printStr("\e[5m", 4); // blinking mode to see cursor
     printStr("\e[2J", 4); // clear
     printStr("\e[H", 4);  // set cursor home
     printStr("Frequency: ", 11);
     printStr("\e[E", 3);
     printStr("Vpp: ", 5);
     printStr("\e[E", 3);
     printStr("DC Offset: ", 11);
     printStr("\e[E", 3);
     printStr("Vrms: ", 6);
     printStr("\e[E", 3);
     printStr("Frequency: ", 11);
     printStr("\e[E", 3);
     printStr("\e[12C", 5);
     printStr("0", 1);
     printStr("\e[7C", 4);
     printStr("250", 3);
     printStr("\e[7C", 4);
     printStr("500", 3);
     printStr("\e[7C", 4);
     printStr("750", 3);
     printStr("\e[6C", 4);
     printStr("1000 Hz", 7);
     printStr("\e[E", 3);
     printStr("Vrms: ", 6);
     printStr("\e[E", 3);
     printStr("\e[12C", 5);
     printStr("0", 1);
     printStr("\e[8C", 5);
     printStr("1", 1);
     printStr("\e[9C", 5);
     printStr("2", 1);
     printStr("\e[9C", 5);
     printStr("3V", 2);
     printStr("\e[H", 4);
     printStr("\e[11C", 5);
}

void update_screen_ac(){
    printStr("\e[H", 4);
    printStr("\e[11C", 5);
    convertDecToAscii((float)freq);
    printStr("Hz", 2);
    printStr("\e[1B", 4);
    printStr("\e[17D", 5);
    convertDecToAscii(convertedPk);
    printStr("Vpp", 3);
    printStr("\e[1B", 4);
    printStr("\e[6D", 4);
    convertDecToAscii(convertedDC);
    printChar('V');
    printStr("\e[1B", 4);
    printStr("\e[15D", 5);
    convertDecToAscii(vrms);
    printChar('V');
    printStr("\e[1B", 4);
    printStr("\e[4D", 4);
    freq_graph((int)freq);
    printStr("\e[E", 4);
    printStr("\e[E", 4);
    printStr("\e[12C", 5);
    voltage_graph((float)vrms);

}

void update_screen_DC(){
    printStr("\e[H", 4);
    printStr("\e[12C", 5);
    convertDecToAscii(convertedDC);
    printStr("V", 1);
    printStr("\e[1B", 4);
    printStr("\e[17D", 5);
    voltage_graph((float)convertedDC);

}

void main(void)
{
//    float convertedDC;
//    float convertedPk;

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    set_DCO(FREQ_48_MHz);                            //set DCO to 48 MHz

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

     TIMER_A0->CCR[0] = 60200;      // Period = 1 sec
     TIMER_A0->CCR[1] = 1000; // Fix value in sampleData() function

     TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE;      // Enable interrupts on TIMER_A0

     NVIC->ISER[0] = 1 << (TA0_0_IRQn & 31);      // Enable CCR0 ISR
     NVIC->ISER[0] = 1 << (TA0_N_IRQn & 31);      // Enable CCR1 ISR


     P3->DIR &= ~BIT0; //P3.0 used for taking in waveform from comparator and calculating frequency
     P3->IES |= BIT0; //set interrupt on high to low transition
     P3->IFG &= ~BIT0; //clear flag
     P3->IE |= BIT0; //enable GPIO interrupts

     NVIC->ISER[1] = 1 << (PORT3_IRQn & 31);

     P6->SEL0 |= BIT1;
     P6->SEL1 |= BIT1; //use P6.1 for ADC input

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

     //screen_init_ac();

     if(state==1)
         screen_init_ac();
     else
         screen_init_DC();

     while(1){
         if(state==1){
              update_screen_ac();
         }
         else{
             update_screen_DC();
         }
         sampleData();
         convertedDC = (max + min)/2/DC_DIVIDER;
         convertedPk = (max - min)/PK_DIVIDER;
         vrms = sqrt(vrms/NUM_SAMPLES);

         P3->IE |= BIT0;
     }
}

