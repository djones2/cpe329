#include "msp.h"
#include "set_DCO.h"

/**
 * main.c
 */

volatile int count = 0;

void screen_init_start();
void screen_init_play();

void printChar(char letter)
{
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

void EUSCIA0_IRQHandler(void)
{
    char letter;

    if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG)
    {
        EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;    //clear the interrupt flag

        // get user input
        letter = EUSCI_A0->RXBUF;   //receive character
	    
        if (letter == 'q')
        {
            state = DC;
            screen_init_start();

        }
        else if (letter == 's')
        {
            state = AC;
            screen_init_DC();
        }
    }
}

void screen_init_play(){
     printStr("\e[5m", 4); // blinking mode to see cursor
     printStr("\e[2J", 4); // clear
     printStr("\e[H", 4);  // set cursor home
     printStr("SCORE: ", 8);
     printStr("\e[E", 3);
     printStr("\e[E", 3);
     printStr("Time remaining: ", 17);
     printStr("\e[E", 3);
     printStr("\e[E", 3);
     printStr("- Press 'q' to quit the game -", 32);
     printStr("\e[H", 4);
     printStr("\e[11C", 5);
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
	
	if(state==0)
        	screen_init_start();
        else
          	screen_init_play();

	while(1)
	{
	    read = (P5->IN & BIT1);
	}
}
