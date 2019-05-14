#include "msp.h"
float v=0;
int interrupt_flag=1;
float vout;
void UART0_init(void);
int i;
int nextval;
char varray[30];

void get_output(int);
void EUSCIA0_IRQHandler(void);

int main(void) {
    //volatile unsigned int i;

    WDT_A->CTL = WDT_A_CTL_PW |             // Stop WDT
                 WDT_A_CTL_HOLD;

    // GPIO Setup
    P1->OUT &= ~BIT0;                       // Clear LED to start
    P1->DIR |= BIT0;                        // Set P1.0/LED to output
    P5->SEL1 |= BIT4;                       // Configure P5.4 for ADC
    P5->SEL0 |= BIT4;

    // Enable global interrupt
    __enable_irq();

    // Enable ADC interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((ADC14_IRQn) & 31);

    // Sampling time, S&H=16, ADC14 on
    ADC14->CTL0 = ADC14_CTL0_SHT0_2 | ADC14_CTL0_SHP | ADC14_CTL0_ON;
    ADC14->CTL1 = ADC14_CTL1_RES_3;         // Use sampling timer, 12-bit conversion results

    ADC14->MCTL[0] |= ADC14_MCTLN_INCH_1;   // A1 ADC input select; Vref=AVCC
    ADC14->IER0 |= ADC14_IER0_IE0;          // Enable ADC conv complete interrupt

 //   SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;   // Wake up on exit from ISR

    __disable_irq();

    UART0_init();

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

    while (1)
    {
        for (i = 20000; i > 0; i--);        // Delay

        __disable_irq();
        // Start sampling/conversion
        if(vflag==1){
        vflag=0;
        vout=(v*(3.3/16560));
        //sprintf(varray, "%f", vout);

        //get_output(v);


        varray[0]=(v/5018);
        varray[1]= '.';
        nextval=(v-(5018*varray[0]));
        varray[2]= (nextval/502);
        nextval = (nextval-502*varray[2]);
        varray[3]= (nextval/50);
        nextval = (nextval-50*varray[3]);
        varray[4]= (nextval/5);

        while(!(EUSCI_A0->IFG & 0x02)) { }  /* wait for transmit buffer empty */
                EUSCI_A0->TXBUF = (varray[0]+48);              /* send a char */
                while(!(EUSCI_A0->IFG & 0x02)) { }  /* wait for transmit buffer empty */
                EUSCI_A0->TXBUF = (varray[1]);              /* send a char */
                while(!(EUSCI_A0->IFG & 0x02)) { }  /* wait for transmit buffer empty */
                EUSCI_A0->TXBUF = (varray[2]+48);              /* send a char */
                while(!(EUSCI_A0->IFG & 0x02)) { }  /* wait for transmit buffer empty */
                EUSCI_A0->TXBUF = (varray[3]+48);              /* send a char */
                while(!(EUSCI_A0->IFG & 0x02)) { }  /* wait for transmit buffer empty */
                EUSCI_A0->TXBUF = (varray[4]+48);              /* send a char */
                while(!(EUSCI_A0->IFG & 0x02)) { }  /* wait for transmit buffer empty */
                EUSCI_A0->TXBUF = (varray[5]+48);              /* send a char */
                EUSCI_A0-> TXBUF = 0x0D;
                ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;

        }

        else{
        interrupt_flag=1;
        }
        __enable_irq();
    }
}

// ADC14 interrupt service routine
void ADC14_IRQHandler(void) {
    if ((ADC14->MEM[0] != v) & (interrupt_flag==0)){        // ADC12MEM0 = A1 > 0.5AVcc?
        v = (ADC14->MEM[0]);                                //(3.3/16384)
        interrupt_flag = 1;
    }


}

void UART0_init(void) {
    EUSCI_A0->CTLW0 |= 1;     /* put in reset mode for config */
    EUSCI_A0->MCTLW = 0;      /* disable oversampling */
    EUSCI_A0->CTLW0 = 0x0081; /* 1 stop bit, no parity, SMCLK, 8-bit data */
    EUSCI_A0->BRW = 26;       /* 3000000 / 115200 = 26 */
    P1->SEL0 |= 0x0C;         /* P1.3, P1.2 for UART */
    P1->SEL1 &= ~0x0C;
    EUSCI_A0->CTLW0 &= ~1;    /* take UART out of reset mode */
    EUSCI_A0->IE |= 1;        /* enable receive interrupt */
}

