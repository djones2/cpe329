/*
 * keypad.h
 *
 *  Created on: Apr 17, 2019
 *      Author: Amanda
 */
#include <msp432.h>
#include <msp432p401r.h>

#ifndef KEYPAD_H_
#define KEYPAD_H_

#define ROW1 BIT0
#define ROW2 BIT1
#define ROW3 BIT2
#define ROW4 BIT4
#define COL1 BIT5
#define COL2 BIT6
#define COL3 BIT7
#define NULLCHAR 0xFF

void keypad_init();
uint8_t key_press();
//void keypad_to_LCD();



#endif /* KEYPAD_H_ */
