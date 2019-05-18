#include "msp.h"
#include <stdio.h>
#include "set_DCO.h"

void TA0_0_IRQHandler(void){
    P6->OUT |= BIT0;
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;      //clears the interrupt flag
    P5->OUT ^=BIT0;                               //toggle LED
    TIMER_A0->CCR[0] += 480;
    P6->OUT &= ~BIT0;
}

void TA0_N_IRQHandler(void){

    if(TIMER_A0->CCTL[1] & TIMER_A_CCTLN_CCIFG){

        TIMER_A0->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG;  //clears the interrupt flags
        P5->OUT |= BIT0;                            //turns on the LED
        TIMER_A0->CCR[1] += 960;
    }

}

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    //set MCLK on 4.3
    P4->DIR |= BIT3;
    P4->SEL0 |= BIT3;
    P4->SEL1 &= ~BIT3;


    set_DCO(FREQ_24_MHz);

    // Set P5.0 as output
    P5->SEL0 &= ~BIT0;
    P5->SEL1 &= ~BIT0;
    P5->DIR |= BIT0;
    P6->SEL0 &= ~BIT0;
    P6->SEL1 &= ~BIT0;
    P6->DIR |= BIT0;

    //P5->OUT |= BIT0;

    // Use SMCLK in continuous mode
    TIMER_A0->CTL |= (TIMER_A_CTL_TASSEL_2|TIMER_A_CTL_MC_2);

     // Period = 40 uS
     // High time = 10us; Low time = 30 uS
     // Change these values for different duty cycles
     TIMER_A0->CCR[0] = 480;
     //TIMER_A0->CCR[1] = 960;

     // Enable interrupts on TIMER_A0
     TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE;
     //TIMER_A0->CCTL[1] = TIMER_A_CCTLN_CCIE;

     // Enable CCR0/1 ISR
     NVIC->ISER[0] = 1 << (TA0_0_IRQn & 31);
     //NVIC->ISER[0] = 1 << (TA0_N_IRQn & 31);

     // Enable global interrupts
     __enable_irq();

     while(1);

}

