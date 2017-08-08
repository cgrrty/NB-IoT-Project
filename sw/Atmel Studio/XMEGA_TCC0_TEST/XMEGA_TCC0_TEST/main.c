/*
 * XMEGA_TCC0_TEST.c
 *
 * Created: 19/05/2017 12:45:40
 * Author : jan.rune.herheim
 */ 

#include <avr/io.h>


int main(void)
{
	PMIC.CTRL |= PMIC_LOLVLEN_bm;
	
	TCC0.INTCTRLA = TC_OVFINTLVL_LO_gc;
    /* Replace with your application code */
    while (1) 
    {
    }
}

