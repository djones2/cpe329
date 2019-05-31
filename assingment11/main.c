#include "msp.h"
#include "keypad.h"
#include "delay.h"
#include "set_DCO.h"


/**
 * main.c
 */

void TA0_0_IRQHandler(void)
{
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;      //clears the interrupt flag
    P5->OUT |= BIT0;                               //toggle LED
    TIMER_A0->CCR[0] += 60000;
}

void TA0_N_IRQHandler(void){

    if(TIMER_A0->CCTL[1] & TIMER_A_CCTLN_CCIFG)
    {
        TIMER_A0->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG;  //clears the interrupt flags
        P5->OUT &= ~BIT0;                            //turns on the LED
        TIMER_A0->CCR[1] += 60000;
    }

}

void main(void)
{
    uint8_t c;
    int tens_digit;
    uint8_t input[2];
    int degrees;

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

    set_DCO(FREQ_3_MHz);

    CS->KEY = CS_KEY_VAL; // unlock CS registers
    CS->CTL1 |= CS_CTL1_DIVS_0 | CS_CTL1_SELS_3; //set SELS and DIVS to source DCO and divide by 1
    CS->KEY = 0; // lock the CS registers

    P5->DIR |= BIT0;
    P5->SEL0 &= ~BIT0;
    P5->SEL1 &= ~BIT0; //P5.0 output GPIO

    //initialize TimerA
    TIMER_A0->CCTL[0] |= TIMER_A_CCTLN_CCIE; // TACCR0 interrupt enabled
    TIMER_A0->CCR[0] = 60000; //20 ms
    TIMER_A0->CCR[1] = 4500;  //1.5 ms
    TIMER_A0->CTL |= TIMER_A_CTL_SSEL__SMCLK | // SMCLK, up mode
                        TIMER_A_CTL_MC__CONTINUOUS;
    TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE;
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_CCIE;

    NVIC->ISER[0] = 1 << ((TA0_0_IRQn) & 31);
    NVIC->ISER[0] = 1 << ((TA0_N_IRQn) & 31);
    __enable_irq();

	keypad_init();

	tens_digit = 1;
	input[0] = '0';
	input[1] = '0';

	while(1)
	{
	    c = key_press();
	    if(c == '*')
	    {
	        //move 10 degrees clockwise
	    }
	    else if(c == '#')
	    {
	        //move 10 degrees counter clockwise
	    }
	    else
	    {
	        if(tens_digit == 1)
	        {
	            input[1] = c;
	            delay_us(4000);
	            tens_digit = 0;
	        }
	        else
	        {
	            input[0] = c;
	            delay_us(4000);
	            //tens_digit = 0;
	        }
	    }
	    if(tens_digit == 0)
	    {
	        tens_digit = 1;
	        //convert ascii to decimal for input[]
	        degrees = input[1] - '0';
	        degrees *=10;
	        degrees += input[0] - '0';
	        CCR[1] += degrees; //will need to change
	    }
	}
}


