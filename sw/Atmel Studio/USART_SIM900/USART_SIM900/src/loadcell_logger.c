/*
 * CFile1.c
 *
 * Created: 12/07/2017 18:53:30
 *  Author: jan.rune.herheim
 */ 

#include "loadcell_logger.h"

uint8_t loadcell_min_max_tran(uint16_t current_value, uint16_t *data_array) {
	//USING THE POSITIONS DEFINED IN THE LOADCELL HEADER FILE.
	
	//find tran
	signed int tran = 0; //could go positive and negative, and could store a 15 bits number, hence enough for our 12 bits results.
	uint16_t tran_abs = 0;
	
	tran = current_value - *(data_array + POSITION_PREV); //tran = current - previous
	if ((abs(tran) > abs(*(data_array+POSITION_TRAN_MAX))) & (*(data_array+POSITION_ACCU_CNT) > 0)) //first step is not valid due to only one value.
	{
		if (tran < 0)
		{
			tran_abs = abs(tran); //check if >2047, if yes this is the max limit that could be transferred.
			if (tran_abs >= 0x7ff) //if yes the set to max value
			{
				tran_abs = 0x7ff;
			}
			tran_abs |= (1<<11); //flip the MSB of the 12 bits word to set the negative sign.
			tran = tran_abs;
		}
		
		*(data_array+POSITION_TRAN_MAX) = tran; //store new tran max.
	}
	
	//find min and max
	if (current_value < *(data_array+POSITION_MIN))
	{
		*(data_array+POSITION_MIN) = current_value; //store new min value.
	}
	if (current_value > *(data_array+POSITION_MAX))
	{
		*(data_array+POSITION_MAX) = current_value; //Store new max.
	}
}
