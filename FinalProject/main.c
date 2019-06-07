#include "msp.h"
#include "set_DCO.h"
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "delay.h"
/**
 * main.c
 */

int score_flag = 0;
volatile int count = 0;
volatile uint32_t width;
volatile int score = 0;
volatile int high_score = 0;
volatile int previous_score = 0;
volatile int bonus = 0;
volatile int timer;
enum State {start = 0, play = 1};
char state;

void screen_init_start();
void screen_init_play();
void update_screen_play();
void update_screen_start();

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

    printStr(str, 2);
    printStr("\e[2C", 4);
}

convertToString(int value)
{
    char str[3];
    str[0] = value/10 + '0';
    str[1] = value%10 + '0';
    str[2] = '\0';
    printStr(str, 2);
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
        if (letter == 'q')
        {
            state = start;
            screen_init_start();
            score = 0;
            timer = 60;
 //           update_screen_start();
        }
        else if (letter == 's')
        {
            score = 0;
            state = play;
            screen_init_play();
        }

        //printChar(letter);
    }
}

void TA0_0_IRQHandler()
{
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;          //clear the interrupt flag
    if(count == 800)
    {
        if (score_flag > 0){
           score++;
           score_flag = 0;
        }

        timer --;
        update_screen_play();
        //timer --;
        count = 0;
        if(timer == 0)
        {
            if(score >= 10 && bonus == 0)
            {
                bonus = 1;
                timer += 30; //30 bonus seconds
            }
            else{
                if(score > high_score)
                {
                    high_score = score;
                }
                previous_score = score;
                bonus = 0;
                score = 0;
                timer = 60;
                //update_screen_start();
                screen_init_start();
                //update_screen_start();
            }

        }
    }
    else
    {
        count ++;
    }
}


void screen_init_play(){
     //count = 0;
     //printStr("\e[5m", 4); // blinking mode to see cursor
     printStr("\e[2J", 4); // clear
     printStr("\e[H", 4);  // set cursor home
     printStr("SCORE: ", 8);
     printStr("\e[E", 3);
     printStr("\e[E", 3);
     printStr("Time remaining: ", 17);
     printStr("\e[E", 3);
     printStr("\e[E", 3);
     printStr("- Press 'q' to quit the game -", 31);
     printStr("\e[H", 4);
     printStr("\e[9C", 5);
     delay_us(3000000); // start timer in 3 seconds
     TIMER_A0->CCTL[0] |= TIMER_A_CCTLN_CCIE; // TACCR0 interrupt enabled (start timer)
}

void screen_init_start(){
    printStr("\e[2J", 4);
//    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIE; // TACCR0 interrupt disabled (disable timer)
     printStr("\e[H", 4);  // set cursor home
     printStr("WELCOME TO DESKTOP BASKETBALL!", 31);
     printStr("\e[E", 3);
     printStr("\e[E", 3);
     printStr("HIGH SCORE: ", 13);
     printStr("\e[E", 3);
     printStr("\e[E", 3);
     printStr("LAST SCORE: ", 13);
     printStr("\e[E", 3);
     printStr("\e[E", 3);
     printStr("- Press 's' to start the game -", 32);
     printStr("\e[H", 4);
     update_screen_start();
//     printStr("\e[11C", 5);
//     delay_us(3000000); // start timer in 3 seconds
     TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIE; // TACCR0 interrupt disabled (disable timer)
}

void update_screen_play(){
    convertToString(score);
    printStr("\e[1B", 4);
    printStr("\e[1B", 4);
    printStr("\e[5C", 5);
    convertToString(timer);
    printStr("\e[H", 4);
    printStr("\e[9C", 5);

}

void update_screen_start(){
    printStr("\e[1B", 4);
    printStr("\e[1B", 4);
    printStr("\e[12C", 5);
    convertToString(high_score);
    printStr("\e[1B", 4);
    printStr("\e[1B", 4);
    printStr("\e[2D", 5);
    convertToString(previous_score);
    printStr("\e[H", 4);
}


void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    set_DCO(FREQ_48_MHz);

    initUART();

    P5->SEL0 &= ~(BIT1|BIT0);
    P5->SEL1 &= ~(BIT0|BIT1);
    P5->DIR |= BIT0; //set P5.0 to output (for Trig)
    P5->DIR &= ~BIT1; //set P5.1 to input (for Echo)
    P5->REN |= BIT1;
    P5->OUT &= ~BIT1; //enable pull down resistor

    TIMER_A0->CTL |= TIMER_A_CTL_SSEL__SMCLK | // SMCLK, up mode
                            TIMER_A_CTL_MC__UP;
    TIMER_A0->CCR[0] = 60000;


    //TIMER_A0->CCTL[0] |= TIMER_A_CCTLN_CCIE; // TACCR0 interrupt enabled

    // Enable CCR0/1 ISR
    NVIC->ISER[0] = 1 << (TA0_0_IRQn & 31);

    // Enable global interrupts
    __enable_irq();
    timer = 60;
    printStr("\e[2J", 4);
    if(state==start){
        screen_init_start();
        update_screen_start();
        score = 0;
    }
    else{
    state = play;
        screen_init_play();
   }
    while(1)
    {
        if(state == play){
            //score_flag = 0;
            width = 0;
            P5->OUT |= BIT0;
            delay_us(10);
            P5->OUT &= ~BIT0;
            //__disable_irq();
            delay_us(448);
            while(P5->IN & BIT1)
            {
                width ++;
            }
            if (width < 2000)
                score_flag = 1;
            //__enable_irq();
            delay_us(8000);
        }
    }
}
