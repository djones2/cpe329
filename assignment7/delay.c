/*
 * delay.c
 *
 *  Created on: Apr 22, 2019
 *      Author: Amanda & Daniel
 */

#include "delay.h"
#include <msp432p401r.h>
#include <msp.h>
#include <stdint.h>

void delay_us(uint32_t delay)
{
    int i;
    int scale = delay>>3;
    int new_del = scale*(1<<(CS->CTL0>>16)) -4;
    for(i=65535; i > 65535 - new_del; i--);
}
