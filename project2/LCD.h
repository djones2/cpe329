/*
 * LCD.h
 *
 *  Created on: Apr 22, 2019
 *      Author: Amanda & Daniel
 */
#include <stdint.h>

#ifndef LCD_H_
#define LCD_H_

#define EN BIT7
#define RW BIT6
#define RS BIT5
#define SETUP_FUNCTION 0x03
#define HOME_LCD 0x03
#define FUNCTION_SET 0x28
#define CLEAR_DISPLAY 0x01
#define DISPLAY_ON_BLINK_CURSOR 0x0F
#define FUNC_SET_4_BIT_2_LINE 0x28
#define ENTRY_MODE 0x06
#define UNIT_DELAY 40
#define BUSY_DELAY 80
#define LONG_DELAY 1600
#define SET_LINE_OFFSET 0x80

void command(int delay, uint8_t i);
void write_char(uint8_t c);
void write_data(uint8_t i, int delay);
void set_line(uint8_t line);
void clear_LCD();
void write_string(char* string);
void home_LCD();
void Nybble();
void lcd_init();
void setAddress(uint8_t address);
uint8_t getAddress();
uint8_t read_data(uint8_t address);
#endif /* LCD_H_ */
