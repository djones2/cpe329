/**
 * main.c
 */

#include "msp.h"
#include "set_DCO.h"
#include "delay.h"
#include <stdint.h>


void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// Stop WDT

	set_DCO(FREQ_48_MHz);

	// Set HSMCLK?

	P5->SEL0 |= BIT5 | BIT6 | BIT7; // Enable A/D channel
    P5->SEL1 |= BIT5 | BIT6 | BIT7; // Enable VeREF+ and VeREF-

    ADC14->CTL0 &= ~ADC14_CTL0_ENC; // Disable ADC14

    // Configure ADC14, set sampling time
    ADC14->CTL0 = ADC14_CTL0_ON
                | ADC14_CTL0_SSEL_5
                | ADC14_CTL0_SHP
                | ADC14_CTL0_SHT0_0;

    // Use 14-bit conversion, start at Mem Location 0
    //CHECK START ADD
    ADC->CTL1 = ADC14CSTARTADD | ADC14_CTL1_RES_3;

    // Mem location 0, VRef  setup, A0
    ADC14->MCTL[0] = ADC14_MCTLN_VRSEL_14 | ADC14_MCTLN_INCH_14;

//    ADC14->IER0 = ADC14IE0_1; // CHECK

    ADC14->CTL0 = ADC14_CTL0_ENC; // enable ADC14

//    ADC14->IER0 = (1<<ADC14_IRQn & 31); // enable interrupt in NVIC

//    __enable_irq();

    while(1){
        ADC14->CTL0 |= ADC14_CTL0_SC;
        delay_us(1000);
    }

/*
    void ADC_14IRQ_Handler(void){
        volatile uint16_t readValue;
        readValue = ADC[14]->MEM[0];
        delay_us(1000);
    }
*/
}
