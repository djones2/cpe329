#include "msp.h"
#include "set_DCO.h"
#include <msp432p401r.h>
#include <stdio.h>
#include <stdint.h>
#include "delay.h"
#include "dac.h"
#include "keypad.h"
#include "LCD.h"
#include <stdint.h>
#include <math.h>

#define SQUARE 7
#define SINE 8
#define SAWTOOTH 9
#define MAX_INDEX 1000

int waveform = SQUARE; //default to square wave
volatile int count = 1;
volatile int index = 0;
volatile int data = 0;

int square_wave[1000];
int sine_wave[1000];
int sawtooth_wave[1000];

int dutyCycle = 50; //default to 50% duty cycle
int maxCount = 1000;
int frequency = 500; //default to 100Hz frequency
int period = 256; //period is the constant value that the CCR's will add because in continuous mode

/**
 * main.c
 */

void setSquareWave(int dc);
void setSignWave();
void setSawtoothWave();

void TA0_0_IRQHandler(void) {
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;          //clear the interrupt flag
    //P2->OUT ^= BIT7;
    if(waveform == SQUARE)
    {
       if(index >= MAX_INDEX)
            index = 0;
       else
       {
            index+=count;
       }
       data = square_wave[index];
       driveDAC(data);
    }
    else if(waveform == SINE)
    {
        if(index >= MAX_INDEX -1)
            index = 0;
        else
            index+= count; //TIMER_A0->CCR[0];
        data = sine_wave[index];
        driveDAC(data);
    }
    else if(waveform == SAWTOOTH)
    {
        if(index >= MAX_INDEX - 1)
            index = 0;
        else
            index+= count; //TIMER_A0->CCR[0];
        data = sawtooth_wave[index];
        driveDAC(data);
    }
}

void setFrequency(int frequency)
{
    // when we change the frequency, we're actually changing the way we index into our wave arrays in the ISR
    switch (frequency)
    {
        case 100:
            count = 1;
            set_line(0x40);
            write_string("100 Hz");
            break;
        case 200:
            count = 2;
            set_line(0x40);
            write_string("200 Hz");
            break;
        case 300:
            count = 3;
            set_line(0x40);
            write_string("300 Hz");
            break;
        case 400:
            count = 4;
            set_line(0x40);
            write_string("400 Hz");
            break;
        case 500:
            count = 5;
            set_line(0x40);
            write_string("500 Hz");
            break;
    }
}

void setSquareWave(int dc)
{
    int i = 0;
    int high_index = dc *10;
    for(i = 0; i < high_index; i++) //set high
        square_wave[i] = 4095;
    for(i = high_index; i < MAX_INDEX; i++) //set low
        square_wave[i] = 0;
}

void setSineWave()
{
    int i = 0;
    for(i=0; i<MAX_INDEX; i++)
    {
        sine_wave[i] = (sin((i * M_PI)/500) +1) * 2047; // DOUBLE SIZE OF SINE WAVE TABLE and multiply again by 2048
    }
}

void setSawtoothWave()
{
    int i = 0;
    for(i = 0; i < MAX_INDEX; i++)
        sawtooth_wave[i] = 4 * i;
}

void main(void)
{
    int input;
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    set_DCO(FREQ_48_MHz); //Set SMCLK to 12 MHz

    //set SMCLK
    // set SMCLK to 12MHz.
    CS->KEY = CS_KEY_VAL; // unlock CS registers
    CS->CTL1 |= CS_CTL1_DIVS_2 | CS_CTL1_SELS_3; //set SELS and DIVS
    CS->KEY = 0; // lock the CS registers


    //---for test
    P2->DIR = 0;
    P2->DIR |= BIT7;
    P2->REN |= BIT7;

    // Map Ports 4.0-4.3 for DB4-DB7
    P4->SEL0 &= ~(BIT3|BIT2|BIT1|BIT0);
    P4->SEL1 &= ~(BIT3|BIT2|BIT1|BIT0);
    P4->DIR |= (BIT3|BIT2|BIT1|BIT0);

    // Map Ports 3.5-3.7 for RS, RW, and EN
    P3->SEL0 &= ~(BIT5|BIT6|BIT7);
    P3->SEL1 &= ~(BIT5|BIT6|BIT7);
    P3->DIR |= (BIT5|BIT6|BIT7);

    keypad_init(); //set up keypad using P5.0-5.7 (excluding P5.3)
    lcd_init();

    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SWRST; // Reset serial peripheral
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_CKPL       //Clock inactive for high mode
                     | EUSCI_B_CTLW0_MSB        //MSB first
                     | EUSCI_B_CTLW0_MST        //Master mode for SPI
                     | EUSCI_B_CTLW0_SYNC       //SPI is synchronous
                     | EUSCI_B_CTLW0_UCSSEL_2   //Select SMCLK
                     | EUSCI_B_CTLW0_SWRST;     //Keep peripheral reset
    EUSCI_B0->BRW = 0x01;                 //Divide to keep SMCLK at desired speed
    // interrupt stuff
    TIMER_A0->CCTL[0] |= TIMER_A_CCTLN_CCIE; // TACCR0 interrupt enabled --should it be |= or just =??
    TIMER_A0->CCR[0] = 120;
    TIMER_A0->CTL |= TIMER_A_CTL_SSEL__SMCLK | // SMCLK, up mode
                    TIMER_A_CTL_MC__UP;

    P1->SEL0 |= BIT5|BIT6;                     //Select EUSCI_B0
    P1->SEL1 &= ~(BIT5|BIT6);                  //for SMCLK and SIMO
    P6->DIR |= EUSCI_CS;                           //Set Port 6.5 as output for CS
    EUSCI_B0->CTLW0 &= ~(EUSCI_B_CTLW0_SWRST);  //Activate SPI Peripheral


    NVIC->ISER[0] = 1 << ((TA0_0_IRQn) & 31);
    __enable_irq();

    //initializing all of the waveform arrays
    setSquareWave(50); //default = 50% duty cycle
    setSineWave();
    setSawtoothWave();

    set_line(0x0);
    write_string("SQUARE WAVE");
    set_line(0x40);
    write_string("100 Hz");

    while(1){
        input = key_press();
        if (input =='*' && waveform == SQUARE)
        {
            if(dutyCycle > 10)
                dutyCycle = dutyCycle - 10;
            setSquareWave(dutyCycle);
        }
        else if(input == '0' && waveform == SQUARE)
        {
            dutyCycle = 50;
            setSquareWave(50);
        }
        else if(input == '#' && waveform == SQUARE)
        {
            if(dutyCycle <90)
                dutyCycle = dutyCycle +10;
            setSquareWave(dutyCycle);
        }
        else if(input == '1')
            setFrequency(100);
        else if(input == '2')
            setFrequency(200);
        else if(input == '3')
            setFrequency(300);
        else if(input == '4')
            setFrequency(400);
        else if(input == '5')
            setFrequency(500);
        else if(input == '7')
        {
            waveform = SQUARE;
            clear_LCD();
            //set to default 100Hz 50% duty cycle square wave -- enable second interrupt
            clear_LCD();
            set_line(0x0);
            write_string("SQUARE WAVE");
            setSquareWave(50);
            setFrequency(100);
        }
        else if(input == '8')
        {
            waveform = SINE;
            clear_LCD();
            set_line(0x0);
            write_string("SINE WAVE");
            setFrequency(100);
        }
        else if(input == '9')
        {
            waveform = SAWTOOTH;
            clear_LCD();
            set_line(0x0);
            write_string("SAWTOOTH WAVE");
            setFrequency(100);
        }
    }
}
