#include "msp.h"
#include "set_DCO.h"

/**
 * main.c
 */

volatile int count = 0;

void TA0_0_IRQHandler()
{
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;          //clear the interrupt flag
    P5->OUT &= ~BIT0; // Trig = high for 10 us
    TIMER_A0->CCR[0] += 65535;
}

void TA0_N_IRQHandler()
{

    if(TIMER_A0->CCTL[1] & TIMER_A_CCTLN_CCIFG)
    {
        TIMER_A0->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG;
        if(count == 2)
        {
            P5->OUT |= BIT0; //Trig = low
            count = 0;
        }
        else
            count ++;
    }
    TIMER_A0->CCR[1] += 65535;
}

void main(void)
{
    int read = 0;
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	set_DCO(FREQ_3_MHz);

	P5->SEL0 &= ~(BIT1|BIT0);
	P5->SEL1 &= ~(BIT0|BIT1);
	P5->DIR |= BIT0; //set P5.0 to output (for Trig)
	P5->DIR &= ~BIT1; //set P5.1 to input (for Echo)
	P5->REN |= BIT1;
	P5->OUT &= ~BIT1; //enable pull down resistor


	TIMER_A0->CTL |= TIMER_A_CTL_SSEL__SMCLK | // SMCLK, up mode
	                        TIMER_A_CTL_MC__CONTINUOUS;
	TIMER_A0->CCR[0] = 30;
	TIMER_A0->CCR[1] = 65535; // 10 us trig pulse
	TIMER_A0->CCTL[0] |= TIMER_A_CCTLN_CCIE; // TACCR0 interrupt enabled
	TIMER_A0->CCTL[1] |= TIMER_A_CCTLN_CCIE; // TACCR1 interrupt enabled

	// Enable CCR0/1 ISR
	NVIC->ISER[0] = 1 << (TA0_0_IRQn & 31);
	NVIC->ISER[0] = 1 << (TA0_N_IRQn & 31);

	// Enable global interrupts
	__enable_irq();

	while(1)
	{
	    read = (P5->IN & BIT1);
	}
}
