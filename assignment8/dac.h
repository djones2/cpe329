/*
 * dac.h
 *
 *  Created on: May 1, 2019
 *      Author: tyfarris
 */

#ifndef DAC_H_

#define DAC_H_

#include "msp.h"
//#include "set_DCO.h"
#include <msp432p401r.h>
#include <stdio.h>
#include <stdint.h>
#include "delay.h"
#include "dac.h"

#define GAIN BIT5
#define SHDN BIT4
#define twoVolts 2482
#define ON 1
#define OFF 0
#define EUSCI_CS BIT5
void driveDAC(int data);

#endif /* DAC_H_ */
