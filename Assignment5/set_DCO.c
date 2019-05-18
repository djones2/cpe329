/*
 * set_DCO.c
 *
 *  Created on: Apr 24, 2019
 *      Author: Amanda & Daniel & Ty
 */
#include "set_DCO.h"
#include <msp432p401r.h>
#include <msp.h>
#include <stdint.h>

// Consider omitting all cases except 24 MHz

void set_DCO(int frequency)
{
    if(frequency == FREQ_1_5_MHz)
    {
        // change DC0 from default of 3MHz to 1.5MHz.
        CS->KEY = CS_KEY_VAL; // unlock CS registers
        CS->CTL0 = 0; // clear register CTL0
        CS->CTL0 = CS_CTL0_DCORSEL_0; // set DCO = 1.5 MHz
        // select clock sources
        //CS->CTL1 = CS_CTL1_SELA_2 | CS_CTL1_SELS_3 | CS_CTL1_SELM_3;
        CS->CTL1 |= CS_CTL1_DIVM__1 | CS_CTL1_SELM__DCOCLK; // MCLK source is DCO
        CS->CTL1 |= CS_CTL1_DIVS__1 | CS_CTL1_SELS__DCOCLK; // SMCLK source is DCO
        CS->KEY = 0; // lock the CS registers

    }
    else if(frequency == FREQ_3_MHz)
    {
        //since DCO default is 3MHz, this code may be unnecessary
        CS->KEY = CS_KEY_VAL; // unlock CS registers
        CS->CTL0 = 0; // clear register CTL0
        CS->CTL0 = CS_CTL0_DCORSEL_1; // set DCO = 1.5 MHz
        // select clock sources
        CS->CTL1 = CS_CTL1_SELA_2 | CS_CTL1_SELS_3 | CS_CTL1_SELM_3;
        CS->KEY = 0; // lock the CS registers
    }
    else if(frequency == FREQ_6_MHz)
    {
        // change DC0 from default of 3MHz to 1.5MHz.
        CS->KEY = CS_KEY_VAL; // unlock CS registers
        CS->CTL0 = 0; // clear register CTL0
        CS->CTL0 = CS_CTL0_DCORSEL_2; // set DCO = 1.5 MHz
        // select clock sources
        CS->CTL1 = CS_CTL1_SELA_2 | CS_CTL1_SELS_3 | CS_CTL1_SELM_3;
        CS->KEY = 0; // lock the CS registers
    }
    else if(frequency == FREQ_12_MHz)
    {
        // change DC0 from default of 3MHz to 12MHz.
        CS->KEY = CS_KEY_VAL; // unlock CS registers
        CS->CTL0 = 0; // clear register CTL0
        CS->CTL0 = CS_CTL0_DCORSEL_3; // set DCO = 12 MHz
        // select clock sources
        CS->CTL1 = CS_CTL1_SELA_2 | CS_CTL1_SELS_3 | CS_CTL1_SELM_3;
        CS->KEY = 0; // lock the CS registers
    }

   // Only using this for Assignment 5
    else if(frequency == FREQ_24_MHz)
    {
        // change DC0 from default of 3MHz to 24MHz.
        CS->KEY = CS_KEY_VAL; // unlock CS registers
        CS->CTL0 = CS_CTL0_DCORSEL_4; // set DCO = 24 MHz
        // select clock sources, changing to SMCLK and MCLK
        CS->CTL1 |= CS_CTL1_DIVM__1 | CS_CTL1_SELM__DCOCLK; // MCLK source is DCO
        CS->CTL1 |= CS_CTL1_DIVS__1 | CS_CTL1_SELS__DCOCLK; // SMCLK source is DCO
        CS->KEY = 0; // lock the CS registers
    }

    else if(frequency == FREQ_48_MHz)
    {
        /* Transition to VCORE Level 1: AM0_LDO --> AM1_LDO */
        while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));
         PCM->CTL0 = PCM_CTL0_KEY_VAL | PCM_CTL0_AMR_1;
        while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));
        /* Configure Flash wait-state to 1 for both banks 0 & 1 */
        FLCTL->BANK0_RDCTL = (FLCTL->BANK0_RDCTL &
         ~(FLCTL_BANK0_RDCTL_WAIT_MASK)) | FLCTL_BANK0_RDCTL_WAIT_1;
        FLCTL->BANK1_RDCTL = (FLCTL->BANK0_RDCTL &
         ~(FLCTL_BANK1_RDCTL_WAIT_MASK)) | FLCTL_BANK1_RDCTL_WAIT_1;
        /* Configure DCO to 48MHz, ensure MCLK uses DCO as source*/
        CS->KEY = CS_KEY_VAL ; // Unlock CS module for register access
        CS->CTL0 = 0; // Reset tuning parameters
        CS->CTL0 = CS_CTL0_DCORSEL_5; // Set DCO to 48MHz
        /* Select MCLK = DCO, no divider */
        CS->CTL1 = CS->CTL1 & ~(CS_CTL1_SELM_MASK | CS_CTL1_DIVM_MASK) |
        CS_CTL1_SELM_3;
        CS->KEY = 0; // Lock CS module from unintended accesses
    }
}



