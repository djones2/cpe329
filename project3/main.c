#include <math.h>
#include "msp.h"
#include <stdint.h>


volatile unsigned int SlowToggle_Period = 10000-1;
volatile unsigned int FastToggle_Period = 1000-1;
/*
 * Here are a bunch of global variables that I think could be useful! One
 * of the tough things will be converting a lot of floats to points, see Assignment
 * 9 to see how I did that. There is a lot of swapping that will be needed to compare incoming
 * samples and plently of counters, hopefully this can provide a framework to get started!
 *
 */


char freq_array[5];
float fout1;
int nextval;
int T_nextval;
int C_nextval;
int vpp_nextval;
int UART_flag;
float vout;
int q;
int t1;
int t2;
int f_count;
float f_try = 100;
float diff;
int next_val;
float fout;
char f_array[30];
int min_flag;
char varray[30];
char vpp_varray[30];
char T_varray[30];
int count=0;
int f_next_val;
float average_array[50];
float T_avgarray[50];
float f_avgarray[100];
float f_sum;
float temp_f;
float DC_avgarray[50];
int calc_flag;
float try = 50;
int new_try = 10;
int CCRO = 24000;
int nextval;
int new_count=0;
int new_avg[10];
int T_new_avg[10];
int DC_new_avg[10];
int new_flaggy;
int new_sum=0;
int T_new_sum=0;
int DC_new_sum=0;
float temp_v;
float T_temp_v;
float DC_temp_v;
int new_temp_v;
int T_new_temp_v;
int DC_new_temp_v;
float v = 0;
float DC_v = 0;
float sum=0;
float T_sum=0;
float DC_sum=0;
float new_max = 0;
float new_min = 10000000;
float peak2;
float peak;
float vpp;
int new_vpp;
int output;
int T_output;
int vpp_output;
int i;

void UART0_init(void);
void ADC14_IRQHandler(void);
void EUSCIA0_IRQHandler(void);
void UART0_init(void);
void to_terminal(int msg);
void terminal();
void set_DCO();
void multimeter_calc();
void init_multimeter();
void get_in_freq();

int main(void) {

    // initialization functions
    init_multimeter();
    UART0_init();

    while (1)
    {
        ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;
        multimeter_calc();

   }
}


// Inside the interrupt, the proper
// voltage samples are modified for true rms and calc rms
// and appended into respective arrays to be averaged at the end of the
// function once enough samples have been gathered.

void TA0_0_IRQHandler(void) {
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
    P3->OUT |= BIT0;

    // Count is the number of sample values taken
    // Try is the amount that should be attempted to be taken, kind of like a max sample to see if signal is AC
    if(count<try){
        average_array[count] = 0;
        average_array[count] = ADC14->MEM[0]+20;   //

        get_in_freq(); // freq calc

        T_avgarray[count] = (average_array[count]) * (average_array[count]);                    //TRUE_RMS
        T_avgarray[count] = (T_avgarray[count]) * (3.3/16560);

        DC_avgarray[count] = 0;
        DC_avgarray[count] = ADC14->MEM[0]+20;
        DC_avgarray[count] = DC_avgarray[count];
        DC_avgarray[count] = (DC_avgarray[count]) * (3.3/16560); // 3.3/2^14 to get DC

        T_sum = T_sum + T_avgarray[count];  // I think this should work to get the sum of TRMS values

        sum = sum + average_array[count];   // sum of

        calc_flag=0;

        DC_sum = DC_sum + DC_avgarray[count];
        count++;

    }
    else if(count==try){

        T_temp_v = (T_sum/try);
        temp_v = (sum/try);

        T_sum = 0;
        sum=0;
        count=0;
        calc_flag=1;
        DC_temp_v = (DC_sum/try);
        DC_sum=0;
    }
    P3->OUT &= ~BIT0;
    TIMER_A0->CCR[0] += CCRO;
}


// initialization of the uart communication
void UART0_init(void) {
    EUSCI_A0->CTLW0 |= 1;     /* put in reset mode for config */
    EUSCI_A0->MCTLW = 0;      /* disable oversampling */
    EUSCI_A0->CTLW0 = 0x0081; /* 1 stop bit, no parity, SMCLK, 8-bit data */
    EUSCI_A0->BRW = 1250;       /* 48000000 / 38400 = 1250 */ // Baud rate needs to increase to over 9600, worth testing!
    P1->SEL0 |= 0x0C;         /* P1.3, P1.2 for UART */
    P1->SEL1 &= ~0x0C;
    EUSCI_A0->CTLW0 &= ~1;    /* take UART out of reset mode */
    EUSCI_A0->IE |= 1;        /* enable receive interrupt */
}

// Configure DCO to 48MHz, ensure MCLK uses DCO as source
void set_DCO(){
            CS->KEY = CS_KEY_VAL ; // Unlock CS module for register access
            CS->CTL0 = 0; // Reset tuning parameters
            CS->CTL0 = CS_CTL0_DCORSEL_5; // Set DCO to 48MHz
            /* Select MCLK = DCO, no divider */
            CS->CTL1 = CS->CTL1 & ~(CS_CTL1_SELM_MASK | CS_CTL1_DIVM_MASK) |
            CS_CTL1_SELM_3;
            CS->KEY = 0; // Lock CS module from unintended accesses
    }

// initialization of TIMERA0 interrupts, P5.4 input, interrupt priority,
//ADC14, SPI, and TIMER32 if we want to use it
// snips of set_DCO, TimerA0, ADC14, and SPI from past projects

void init_multimeter(){
    /* Transition to VCORE Level 1: AM0_LDO --> AM1_LDO */
    while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));
     PCM->CTL0 = PCM_CTL0_KEY_VAL | PCM_CTL0_AMR_1;
    while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));

    /* Configure Flash wait-state to 1 for both banks 0 & 1 */
    FLCTL->BANK0_RDCTL = (FLCTL->BANK0_RDCTL &
     ~(FLCTL_BANK0_RDCTL_WAIT_MASK)) | FLCTL_BANK0_RDCTL_WAIT_1;
    FLCTL->BANK1_RDCTL = (FLCTL->BANK0_RDCTL &
     ~(FLCTL_BANK1_RDCTL_WAIT_MASK)) | FLCTL_BANK1_RDCTL_WAIT_1;

    WDT_A->CTL = WDT_A_CTL_PW |             // Stop WDT
                 WDT_A_CTL_HOLD;

    // GPIO Setup
    P1->OUT &= ~BIT0;                       // Clear LED to start
    P1->DIR |= BIT0;                        // Set P1.0/LED to output
    P5->SEL1 |= BIT4 | BIT5;                // Configure P5.4/5 for ADC
    P5->SEL0 |= BIT4 | BIT5;
    P3->DIR |= BIT0;
    P3->OUT |= BIT0;


    TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE; // TACCR0 interrupt enabled
    TIMER_A0->CCR[0] = CCRO;
    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | // SMCLK, continuous mode
            TIMER_A_CTL_MC__CONTINUOUS;

    set_DCO(); // auto sets to 48 MHz

    // Enable global interrupt
    __enable_irq();

    // Enable ADC interrupt in NVIC module
    //NVIC->ISER[0] = 1 << ((ADC14_IRQn) & 31);
    NVIC->ISER[0] = 1 << ((TA0_0_IRQn) & 31);

    // Sampling time, S&H=16, ADC14 on
    ADC14->CTL0 = ADC14_CTL0_SHT0_2 | ADC14_CTL0_SHP | ADC14_CTL0_ON;
    ADC14->CTL1 = ADC14_CTL1_RES_3;         // Use sampling timer, 12-bit conversion results

    ADC14->MCTL[0] |= ADC14_MCTLN_INCH_1;   // A1 ADC input select; Vref=AVCC
    ADC14->IER0 |= ADC14_IER0_IE0;          // Enable ADC conv complete interrupt

    SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;   // Wake up on exit from ISR

    __disable_irq();


    /* initialize P2.2-P2.0 for tri-color LEDs */
    P2->SEL1 &= ~7;           /* configure P2.2-P2.0 as simple I/O */
    P2->SEL0 &= ~7;
    P2->DIR |= 7;             /* P2.2-2.0 set as output */

    NVIC_SetPriority(EUSCIA0_IRQn, 4); /* set priority to 4 in NVIC */
    NVIC_EnableIRQ(EUSCIA0_IRQn);      /* enable interrupt in NVIC */
    __enable_irq();                    /* global enable IRQs */

    P4->DIR |= BIT1;                     // Will use BIT4 to activate /CE on the DAC
    P1SEL0 |= BIT6 + BIT5;               // Configure P1.6 and P1.5 for UCB0SIMO and UCB0CLK
    P1SEL1 &= ~(BIT6 + BIT5);            //

    // SPI Setup
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SWRST;   // Put eUSCI state machine in reset

    EUSCI_B0->CTLW0 = EUSCI_B_CTLW0_SWRST |   // Remain eUSCI state machine in reset
                      EUSCI_B_CTLW0_MST   |   // Set as SPI master
                      EUSCI_B_CTLW0_SYNC  |   // Set as synchronous mode
                      EUSCI_B_CTLW0_CKPL  |   // Set clock polarity high
                      EUSCI_B_CTLW0_MSB;      // MSB first

    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SSEL__SMCLK; // SMCLK
    EUSCI_B0->BRW = 0x01;                         // divide by 16, clock = fBRCLK/(UCBRx)
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;      // Initialize USCI state machine, SPI
                                                  // now waiting for something to
                                                  // be placed in TXBUF

    EUSCI_B0->IFG |= EUSCI_B_IFG_TXIFG;  // Clear TXIFG flag

    // If we want to use Timer 32 for the control register, here it is!

    TIMER32_1->CONTROL = TIMER32_CONTROL_SIZE + TIMER32_CONTROL_ENABLE + TIMER32_CONTROL_PRESCALE_2;
}

// This function averages all of the sampled data to ensure accuracy.
// Calculate peak-to-peak values based on input frequency
// Each digit of the calculated voltage values are translated into respective arrays to be uart
void multimeter_calc(){

    if(calc_flag==1){

/* TO-DO:
 * calling this value will take the input voltage from either 5.4/5.5 and
 * get the RMS value from the ADC input (MEM[0]) and get output values
 * for the terminal to output.
 *
 */


        /*
         * Makes the arrays for DC voltage, TRMS, Vpp
         */
        varray[0]=(output/5018);
        varray[1]= '.';
        nextval=(output-(5018*varray[0]));
        varray[2]= (nextval/502);
        nextval = (nextval-502*varray[2]);
        varray[3]= (nextval/50);
        nextval = (nextval-50*varray[3]);
        varray[4]= (nextval/5);

        T_varray[0]=(T_output/5018);
        T_varray[1]= '.';
        T_nextval=(T_output-(5018*T_varray[0]));
        T_varray[2]= (T_nextval/502);
        T_nextval = (T_nextval-502*T_varray[2]);
        T_varray[3]= (T_nextval/50);
        T_nextval = (T_nextval-50*T_varray[3]);
        T_varray[4]= (T_nextval/5);

        vpp_varray[0]=(vpp_output/5018);
        vpp_varray[1]= '.';
        vpp_nextval=(vpp_output-(5018*vpp_varray[0]));
        vpp_varray[2]= (vpp_nextval/502);
        vpp_nextval = (vpp_nextval-502*vpp_varray[2]);
        vpp_varray[3]= (vpp_nextval/50);
        vpp_nextval = (vpp_nextval-50*vpp_varray[3]);
        vpp_varray[4]= (vpp_nextval/5);

        terminal();


        ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;
    }
}

// Displays to the terminal using VT 100 protocol.
// * character is delineator on bar graph
void terminal(){

    to_terminal('B');
    to_terminal('E');
    to_terminal('S');
    to_terminal('T');
    to_terminal(' ');
    to_terminal('M');
    to_terminal('U');
    to_terminal('L');
    to_terminal('T');
    to_terminal('I');
    to_terminal('M');
    to_terminal('E');
    to_terminal('T');
    to_terminal('E');
    to_terminal('R');
    to_terminal(' ');
    to_terminal('E');
    to_terminal('V');
    to_terminal('E');
    to_terminal('R');
    to_terminal(':');


    to_terminal(10); // starts on new line
    to_terminal(13);
    to_terminal(10); // starts on new line
    to_terminal(13);

    to_terminal('D');
    to_terminal('C');
    to_terminal(' ');
    to_terminal('V');
    to_terminal('O');
    to_terminal('L');
    to_terminal('T');
    to_terminal('A');
    to_terminal('G');
    to_terminal('E');
    to_terminal(':');

    to_terminal(varray[0]+48); //prints DC Value
    to_terminal(varray[1]);
    to_terminal(varray[2]+48);
    to_terminal(varray[3]+48);
    to_terminal(varray[4]+48);


    to_terminal(10); // starts on new line
    to_terminal(13);
    to_terminal(10); // starts on new line
    to_terminal(13);

    to_terminal('V');
    to_terminal('p');
    to_terminal('p');
    to_terminal(':');

    to_terminal(vpp_varray[0]+48); //prints Vpp Value
    to_terminal(vpp_varray[1]);
    to_terminal(vpp_varray[2]+48);
    to_terminal(vpp_varray[3]+48);
    to_terminal(vpp_varray[4]+48);


    to_terminal(10); // starts on new line
    to_terminal(13);
    to_terminal(10); // starts on new line
    to_terminal(13);

    to_terminal('T'); // This will be AC value for true rms
    to_terminal('R');
    to_terminal('M');
    to_terminal('S');
    to_terminal(':');

    to_terminal(T_varray[0]+48); //prints DC Value
    to_terminal(T_varray[1]);
    to_terminal(T_varray[2]+48);
    to_terminal(T_varray[3]+48);
    to_terminal(T_varray[4]+48);


    to_terminal(10); // starts on new line
    to_terminal(13);
    to_terminal(10); // starts on new line
    to_terminal(13);


    to_terminal('F');
    to_terminal('R');
    to_terminal('E');
    to_terminal('Q');
    to_terminal('U');
    to_terminal('E');
    to_terminal('N');
    to_terminal('C');
    to_terminal('Y');
    to_terminal(':');

    to_terminal(f_array[0]+48); //prints freq Value
    to_terminal(f_array[1]+48);
    to_terminal(f_array[2]+48);
    to_terminal(f_array[3]+48);
    to_terminal(f_array[4]+48);

     to_terminal(10); // starts on new line
     to_terminal(13);
     to_terminal(10); // starts on new line
     to_terminal(13);

     to_terminal('D');
     to_terminal('C');
     to_terminal(' ');
     to_terminal('V');
     to_terminal('O');
     to_terminal('L');
     to_terminal('T');
     to_terminal('A');
     to_terminal('G');
     to_terminal('E');
     to_terminal(':');

     int b;
     if((varray[0]==0)&&(varray[2]<10)){
         for(b=0;b<varray[2];b++){
             to_terminal('*');
         }
     }

     if((varray[0]==1)&&(varray[2]<10)){
              for(b=0;b<10;b++){
                  to_terminal('*');
              }
              for(b=0;b<varray[2];b++){
                 to_terminal('*');
              }
     }

     if ((varray[0] == 2)&&(varray[2]<10)){
         for(b=0;b<20;b++){
             to_terminal('*');
         }
         for(b=0;b<varray[2];b++){
             to_terminal('*');
        }
     }

    to_terminal(10); // starts on new line
    to_terminal(13);
    to_terminal(10); // starts on new line
    to_terminal(13);

    to_terminal('T');
    to_terminal('R');
    to_terminal('U');
    to_terminal('E');
    to_terminal(' ');
    to_terminal('R');
    to_terminal('M');
    to_terminal('S');
    to_terminal(':');

    /*
     * For each array, post a * character for
     * each .1 V. Checks if 0.x V, 1.x V, or 2.x/3.x V
     */

    if((T_varray[0]==0)&&(T_varray[2]<10)){
      for(b=0;b<T_varray[2];b++){
          to_terminal('*');
      }
    }

    if((T_varray[0]==1)&&(T_varray[2]<10)){
       for(b=0;b<10;b++){
           to_terminal('*');
       }
       for(b=0;b<T_varray[2];b++){
          to_terminal('*');
       }
    }

    if ((T_varray[0] == 2)&&(T_varray[2]<10)){
      for(b=0;b<20;b++){
          to_terminal('*');
      }
      for(b=0;b<T_varray[2];b++){
          to_terminal('*');
     }
    }

    to_terminal(27);
    to_terminal('[');
    to_terminal('H'); // back to top? check VT100 if you could!
}

// This function individually transmits through the UART. It is repeatedly called in the terminal function
void to_terminal(int msg){

    while(!(EUSCI_A0->IFG & 0x02)) { }  /* wait for transmit buffer empty */

    EUSCI_A0->TXBUF = (msg);

}

// This function will TIMER32 to collect the times at which "high" voltage values are registered
// It then uses them to calculate the frequency
// Use Timer_A/32 to collect time that voltages change from high to low (frequency)
// A State Machine could be used here to decide AC versus DC!
//
void get_in_freq(){


}


