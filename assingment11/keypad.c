/*
 * keypad.c
 *
 *  Created on: Apr 17, 2019
 *      Author: Amanda
 */

#include <stdint.h>
#include <msp.h>
#include <msp432p401r.h>
#include "LCD.h"
#include "keypad.h"
#include "set_DCO.h"
#include "delay.h"
#include <string.h>

void keypad_init(void)
{
    P5->DIR = 0;
    P5->DIR |= (COL1|COL2|COL3);
    P5->REN |= (ROW1|ROW2|ROW3|ROW4);
    P5->OUT &= ~(ROW1|ROW2|ROW3|ROW4);
}


uint8_t key_press()
{
    int row, col, key;
    P5->OUT |= (COL1|COL2|COL3);
    _delay_cycles(25);
    //check if key was pressed
    row = (P5->IN)&(ROW1|ROW2|ROW3|ROW4); //get 4 LSBs to read rows
    if(row == 0)
        return NULLCHAR; //no key pressed
    for(col = 0; col < 3; col++)
    {
        P5->OUT &= ~(COL1|COL2|COL3); //set cols to 0
        P5->OUT |= (COL1 << col); //put a 1 at each col, shifting each time
        _delay_cycles(25);
        row = (P5->IN)&(ROW1|ROW2|ROW3|ROW4);
        if(row != 0) //if detect a key pressed, break out of loop
            break;
    }
    P5->OUT &= ~(COL1|COL2|COL3); //turn off columns
    if(col == 3) //no button press or missed button press
        return NULLCHAR;
    if(row == 4)
        row = 3;
    if(row == 16) //8)
        row = 4;
    key = (row-1)*3 + 1 + col;
    if(key == 11)
        key = 0; //special case for 0
    key += '0'; //turn into ASCII value
    if(key == 0x3A)
        key = '*';
    else if (key == 0x3C)
        key = '#';
    return key;
}

void keypad_to_LCD()
{
    delay_us(40);
    uint8_t c = key_press();
    delay_us(40);
    if(c != NULLCHAR) //if a key was pressed
    {
        delay_us(40000);
        write_char(c);
        delay_us(40000);
        delay_us(40000);
        delay_us(40000);
    }
}


