/*
 * wavegen.c
 *
 *  Created on: May 3, 2019
 *      Author: Amanda
 */
#include "wavegen.h"
#include "dac.h"
#include "msp432.h"

int lookup(int index)
{
    return sinewave[index];
}



