#include "msp.h"
#include "set_DCO.h"
#include <msp432p401r.h>
#include <stdio.h>
#include <stdint.h>
#include "delay.h"
#include "dac.h"
#include "keypad.h"
#include "wavegen.h"
#include <stdint.h>

int waveform = SQUARE; //default to square wave
volatile int count = 0;
volatile int data = 0;
int dutyCycle = 50; //default to 50% duty cycle
int frequency = 100; //default to 100Hz frequency
int period = 256; //period is the constant value that the CCR's will add because in continuous mode

/**
 * main.c
 */

void TA0_0_IRQHandler(void) {
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;          //clear the interrupt flag
    if(waveform == SQUARE)
    {
        data = 0; // Vout = GND
        TIMER_A0->CCR[0] += period;//TIMER_A0->CCR[1]; // add period to CCR[0]
    }
    else if(waveform == SINE)
    {
        data = lookup(count);
        if(count == MAX_INDEX)
            count = 0;
        else
            count+= period; //TIMER_A0->CCR[0];
        //NO IDEA if this is right- trying to get the time between outputs from the DAC to be the same for all frequencies
    }
    else if(waveform == SAWTOOTH)
    {
        data += 2;
        TIMER_A0->CCR[0] += period; //TIMER_A0->CCR[0]; // add CCR[0] to itself since in continuous mode
    }
    driveDAC(data);
}

void TA0_N_IRQHandler(void){
    if(TIMER_A0->CCTL[1] & TIMER_A_CCTLN_CCIFG){
        TIMER_A0->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG;
   //clears the interrupt flags
        data = 4096; // Vout = VDD
        driveDAC(data);
        TIMER_A0->CCR[1] += TIMER_A0->CCR[1]; //add period to CCR[1]
    }
}

void setDutyCycle(int newDC)
{
    dutyCycle = newDC;
    count = 0; //reset count;

}

void setFrequency(int frequency)
{
    if(waveform == SQUARE)
    {
        TIMER_A0->CCR[1] = (FREQ_12_MHz)/frequency;
        TIMER_A0->CCR[0] = dutyCycle * TIMER_A0->CCR[1]; //setting duty cycle for new frequency
        period = TIMER_A0->CCR[1];
    }
    else if(waveform == SINE)
    {
        TIMER_A0->CCR[0] = ((FREQ_12_MHz)/frequency)/4096; //same number of data points as sawtooth?
        period = TIMER_A0->CCR[0];
    }
    else
    {
        TIMER_A0->CCR[0] = ((FREQ_12_MHz)/frequency)/4096; //same number of data points as sawtooth?
        period = TIMER_A0->CCR[0];

    }
}

void decrementDutyCycle()
{
    if (dutyCycle == 10)
            return;
    dutyCycle -= 10;
    count = 0; //reset count
    TIMER_A0->CCR[0] += period + dutyCycle * period; //TIMER_A0_CCR[1]; //duty cycle * period -- no idea if this will work...
    TIMER_A0->CCR[1] += period + (1-dutyCycle)*period;
}

void incrementDutyCycle()
{
    if (dutyCycle == 90)
            return;
    dutyCycle += 10;
    count = 0; //reset count
    TIMER_A0->CCR[0] = period + (dutyCycle * period); //period + duty cycle * period
    TIMER_A0->CCR[1] += period + (1-dutyCycle)*period; //period + (1-duty cycle)*period -- delaying a period to reset..
}


void main(void)
{
    int input;
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    set_DCO(FREQ_12_MHz); //Set SMCLK to 12 MHz

    // Map Ports 4.0-4.3 for DB4-DB7 on keypad
    P4->SEL0 &= ~(BIT3|BIT2|BIT1|BIT0);
    P4->SEL1 &= ~(BIT3|BIT2|BIT1|BIT0);
    P4->DIR |= (BIT3|BIT2|BIT1|BIT0);
    keypad_init();

    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SWRST; // Reset serial peripheral
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_CKPL       //Clock inactive for high mode
                     | EUSCI_B_CTLW0_MSB        //MSB first
                     | EUSCI_B_CTLW0_MST        //Master mode for SPI
                     | EUSCI_B_CTLW0_SYNC       //SPI is synchronous
                     | EUSCI_B_CTLW0_UCSSEL_2   //Select SMCLK
                     | EUSCI_B_CTLW0_SWRST;     //Keep peripheral reset
    EUSCI_B0->BRW = 0x01;                 //Divide to keep SMCLK at desired speed
    // interrupt stuff
    TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE; // TACCR0 interrupt enabled --should it be |= or just =??
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_CCIE; // TACCR1 interrupt enabled
    TIMER_A0->CCR[0] = 128;
    TIMER_A0->CCR[1] = 256; //I think this sets the SQUARE wave to 100Hz (default) with 50% duty cycle (default)
    period = 256;
    TIMER_A0->CTL |= TIMER_A_CTL_SSEL__SMCLK | // SMCLK, continuous mode
                    TIMER_A_CTL_MC__CONTINUOUS;
    P1->SEL0 |= BIT5|BIT6;                     //Select EUSCI_B0
    P1->SEL1 &= ~(BIT5|BIT6);                  //for SMCLK and SIMO
    P6->DIR |= EUSCI_CS;                           //Set Port 6.5 as output for CS
    EUSCI_B0->CTLW0 &= ~(EUSCI_B_CTLW0_SWRST);  //Activate SPI Peripheral
    __enable_irq();

    NVIC->ISER[0] = 1 << ((TA0_0_IRQn) & 31);
    NVIC->ISER[0] = 1 << (TA0_N_IRQn & 31);

    while(1){
        input = key_press();
        /*if (input != NULLCHAR && input !='6')
        {
            keypad_to_LCD();
            delay_us(4000);
            count = 0; //reset count for new waveform/frequency/duty cycle
        }*/
        if (input =='*' && waveform == SQUARE)
            decrementDutyCycle();
        else if(input == '0' && waveform == SQUARE)
            setDutyCycle(50);
        else if(input == '#' && waveform == SQUARE)
            incrementDutyCycle();
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
            //set to default 100Hz 50% duty cycle square wave -- enable second interrupt
            TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE; // TACCR0 interrupt enabled --should it be |= or just =??
            TIMER_A0->CCTL[1] = TIMER_A_CCTLN_CCIE; // TACCR1 interrupt enabled
            TIMER_A0->CCR[0] = 128;
            TIMER_A0->CCR[1] = 256; //I think this sets the SQUARE wave to 100Hz (default) with 50% duty cycle (default)
        }
        else if(input == '8')
        {
            waveform = SINE;
            TIMER_A0->CCTL[1] = ~TIMER_A_CCTLN_CCIE; // TACCR1 interrupt disabled
        }
        else if(input == '9')
        {
            waveform = SAWTOOTH;
            TIMER_A0->CCTL[1] = ~TIMER_A_CCTLN_CCIE; // TACCR1 interrupt disabled
        }

    }
}
