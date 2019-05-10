/*
 * set_DCO.h
 *
 *  Created on: Apr 24, 2019
 *      Author: Amanda & Daniel & Ty
 */

#ifndef SET_DCO_H_
#define SET_DCO_H_


#define FREQ_1_5_MHz  1500000   //CS_CTL0_DCORSEL_0
#define FREQ_3_MHz    3000000   //CS_CTL0_DCORSEL_1
#define FREQ_6_MHz    6000000   //CS_CTL0_DCORSEL_2
#define FREQ_12_MHz   12000000  //CS_CTL0_DCORSEL_3
#define FREQ_24_MHz   24000000  //CS_CTL0_DCORSEL_4
#define FREQ_48_MHz   48000000  //CS_CTL0_DCORSEL_5

void set_DCO(int freq);


#endif /* SET_DCO_H_ */
