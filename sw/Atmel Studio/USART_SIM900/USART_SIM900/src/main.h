/*
 * main.h
 *
 * Created: 28/06/2017 17:09:19
 *  Author: jan.rune.herheim
 */ 


#ifndef MAIN_H_
#define MAIN_H_

#include "asf.h"
#include "string.h"
#include "avr/interrupt.h"
#include "math.h"

//COMMUNICATION PACKAGE SETUP
//COULD BE UPC UA OR OTHER.
#include "MQTT_functions.h"

//USART
#include "conf_usart_example.h"

//RADIO SETUP
#include "sim900_at_commands.h"

//EXTERNAL SENSOR SETUP
#include "loadcell_logger_setup.h"

//function declarations:
void reset_char_array(char *array_pointer , uint8_t size);


#endif /* MAIN_H_ */