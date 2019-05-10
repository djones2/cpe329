/**

 * main.c

 */



#include "msp.h"

#include "set_DCO.h"

#include "delay.h"

#include <stdint.h>


volatile uint16_t interrupt_flag = 0;


void main(void)

{

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // Stop WDT

    set_DCO(FREQ_48_MHz);

    // Set HSMCLK
    CS->KEY = CS_KEY_VAL;       // unlock CS registers
    CS->CTL1 |= CS_CTL1_SELS_3; //set HSMCLK to DCO
    CS->KEY = 0;            //lock CS registers

    P4->DIR |= BIT0; //P4.0 output
    P4->SEL0 &= ~BIT0; //P4.0 GPIO
    P4->SEL1 &= ~BIT0; //P4.0 GPIO

    P5->SEL0 |= BIT5 | BIT6 | BIT7; // Enable A/D channel
    P5->SEL1 |= BIT5 | BIT6 | BIT7; // Enable VeREF+ and VeREF-

    ADC14->CTL0 &= ~ADC14_CTL0_ENC; // Disable ADC14

    // Configure ADC14, set sampling time
    ADC14->CTL0 = ADC14_CTL0_ON     //turn on ADC
                    | ADC14_CTL0_SSEL_5 //select HSMCLK
                    | ADC14_CTL0_SHP    //sample pulse mode
                    | ADC14_CTL0_SHT0_0;    //select 4 clock cycles

    // Use 14-bit conversion, start at Mem Location 0
    ADC14->CTL1 = ADC14_CTL1_CSTARTADD_OFS | ADC14_CTL1_RES_3; //16 clock cycle conversion time

    // Mem location 0, VRef  setup, A0
    ADC14->MCTL[0] = ADC14_MCTLN_VRSEL_14 | ADC14_MCTLN_INCH_14;
    ADC14->IER0 |= ADC14_IER0_IE0; // CHECK to enable interrupt
    ADC14->CTL0 |= ADC14_CTL0_ENC; // enable ADC14

    NVIC->ISER[0] = (1<<(ADC14_IRQn & 31)); // enable interrupt in NVIC
    __enable_irq();

    ADC14->CTL0 |= ADC14_CTL0_SC;

    while(1)
    {
        if(interrupt_flag == 1)
        {
            interrupt_flag = 0;
            ADC14->CTL0 |= ADC14_CTL0_SC;
            delay_us(1000); // Set a breakpoint!!
        }
    }
}

void ADC14_IRQHandler(void)
{
    volatile uint16_t readValue;
    while(!(ADC14->IFGR0 & BIT0));
    readValue = ADC14->MEM[0];
    if(readValue)
    {
        P4->OUT |= BIT0;
    }
    else
    {
        P4->OUT &= ~BIT0;
    }
    interrupt_flag = 1;
}
