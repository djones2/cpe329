/**
 * main.c
 */

#include "msp.h"
#include "set_DCO.h"
#include "delay.h"
#include <stdint.h>
#include "uart.c"

volatile uint16_t interrupt_flag = 0;

int main(void) {

    set_DCO(FREQ_48_MHz);

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // Stop WDT

    // Set HSMCLK
    CS->KEY = CS_KEY_VAL;       // unlock CS registers
    CS->CTL1 |= CS_CTL1_SELS_3; //set HSMCLK to DCO
    CS->KEY = 0;                //lock CS registers

    P5->SEL1 |= BIT5;          //P5.5 for analog read
    P5->SEL0 |= BIT5;

    ADC14->CTL0 &= ~ADC14_CTL0_ENC; // Disable ADC14

    // Configure ADC14, set sampling time
    ADC14->CTL0 = ADC14_CTL0_ON             //turn on ADC
                | ADC14_CTL0_SSEL_5     //select HSMCLK
                | ADC14_CTL0_SHP        //sample pulse mode
                | ADC14_CTL0_SHT0_7;    //select 4 clock cycles

    // Use 14-bit conversion, start at Memory Location 0
    ADC14->CTL1 = ADC14_CTL1_CSTARTADD_OFS  //memory location 0
                | ADC14_CTL1_RES_3;         //14 bit conversion

    // Mem location 0, VRef  setup, A0
    ADC14->MCTL[0] = ADC14_MCTLN_INCH_0;
    ADC14->IER0 |= ADC14_IER0_IE0;          // enable interrupt
    ADC14->CTL0 |= ADC14_CTL0_ENC;          // enable ADC14

 //   uartINIT();

    NVIC->ISER[0] = (1<<(ADC14_IRQn & 31)); // enable interrupt in NVIC

    __enable_irq();


 //   ADC14->CTL0 |= ADC14_CTL0_SC;

    while(1)
    {
        ADC14->CTL0 |= ADC14_CTL0_SC;
        if(interrupt_flag == 1)
        {
            interrupt_flag = 0;
            ADC14->CTL0 |= ADC14_CTL0_SC;
 //           delay_us(1000);
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
            P5->OUT |= BIT5;
        }
        else
        {
            P5->OUT &= ~BIT5;
        }
    interrupt_flag = 1;
}


