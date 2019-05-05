/*
 * LCD.c
 *
 *  Created on: Apr 22, 2019
 *      Author: Amanda & Daniel
 */

//4-bit Initialization:
/**********************************************************/
#include "delay.h"
#include "set_DCO.h"
#include <msp432p401r.h>
#include <stdint.h>
#include <msp.h>
#include "LCD.h"
#include <string.h>

void write_data(uint8_t i, int delay);
void Nybble();
void clear_LCD();
void home_LCD();
void command(int delay, uint8_t i)
{
 P3->OUT &= ~(EN|RS|RW); //D/I=LOW, R_W=LOW : send instruction, Write
 P4->OUT = (i>>4); //put data on output Port
 Nybble(); //Send lower 4 bits
 P4->OUT = i; //put data on output Port
 Nybble(); //Send upper 4 bits
 delay_us(delay);
}

void set_line(uint8_t line)
{
    command(UNIT_DELAY, SET_LINE_OFFSET|line);
}

void write_char(uint8_t c)
{
    P3->OUT |= RS; //RS high
    write_data(c, UNIT_DELAY);
}

/**********************************************************/
void write_data(uint8_t i, int delay)
{
 P4->OUT = (i>>4); //put data on output Port
 P3->OUT |= (RS); //D/I=HIGH: send data
 P3->OUT &= ~(EN|RW); //EN low, RW low: write
 Nybble(); //Clock lower 4 bits
 P4->OUT = i; //put data on output Port
 Nybble(); //Clock upper 4 bits
 delay_us(delay);
}

void clear_LCD()
{
    P3->OUT &= ~(RS|RW);
    command(LONG_DELAY, CLEAR_DISPLAY);
}

void home_LCD()
{
    P3->OUT &= ~(RS);
    //P3->OUT |= RW;
    command(LONG_DELAY, HOME_LCD);
}

void write_string(char* string)
{
    int i;
    for(i = 0; i < strlen(string); i++)
    {
        write_char(string[i]);
    }
}

void Nybble()
{
 P3->OUT |= EN;
 delay_us(0); //enable pulse width >= 300ns
 P3->OUT &= ~EN; //Clock enable: falling edge
}

void lcd_init()
{
 delay_us(45000); //Wait >40 msec after power is applied
 P4->OUT = 0;
 P3->OUT &= ~(EN|RW|RS);
 P4->OUT = SETUP_FUNCTION;
 Nybble();
 delay_us(UNIT_DELAY); //must wait 5ms, busy flag not available
 command(UNIT_DELAY, FUNCTION_SET); //Function set: 4-bit/2-line
 command(UNIT_DELAY, FUNCTION_SET); //Function set: 4-bit/2-line
 delay_us(BUSY_DELAY);
 command(UNIT_DELAY, DISPLAY_ON_BLINK_CURSOR); //Display ON; Blinking cursor
 command(LONG_DELAY, CLEAR_DISPLAY);
 command(UNIT_DELAY, ENTRY_MODE); //Entry Mode set and direction

}

void setAddress(uint8_t address) {
    command(0x80|address, 40);
}



uint8_t read_data(uint8_t address) {
    uint8_t upper, lower;

    // Set address to read input data from
    setAddress(address);

    P3->OUT |= RS;                         // RS high
    P3->OUT |= RW;                         // RW high
    P3->OUT &= ~EN;
    P4->DIR &= ~(BIT3|BIT2|BIT1|BIT0);     // Direction to input
    P3->OUT |= EN;
    upper = P4->IN & (BIT3|BIT2|BIT1|BIT0); // Upper 4 bits of address
    P3->OUT &= ~EN;
    P3->OUT |= EN;
    lower = P4->IN & (BIT3|BIT2|BIT1|BIT0); // Lower 4 bits of address
    P3->OUT &= ~EN;
    P4->DIR |= BIT3|BIT2|BIT1|BIT0;         // Direction to output
    return (upper << 4) | lower;
}

uint8_t getAddress() {

    uint8_t upper, lower;

    P3->OUT &= ~RS;                         // RS low
    P3->OUT |= RW;                          // RW high
    P3->OUT &= ~EN;
    P4->DIR &= ~(BIT3|BIT2|BIT1|BIT0);      // Direction to input
    P3->OUT |= EN;                          // Upper 4 bits of addrress
    upper = P4->IN & (BIT2|BIT1|BIT0);
    P3->OUT &= ~EN;
    P3->OUT |= EN;                          // Lower 4 bits of address
    lower = P4->IN & (BIT3|BIT2|BIT1|BIT0);
    P3->OUT &= ~EN;
    P4->DIR |= BIT3|BIT2|BIT1|BIT0;         // Direction to output
    return (upper << 4) | lower;

}
