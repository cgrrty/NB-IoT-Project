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
#include "avr/wdt.h"
#include "avr/sleep.h"



//USART
//#include "conf_usart_example.h"
#include "ASF/mega/drivers/usart_mega.h"

//RADIO SETUP
#include "quectel_m95.h"
//#include "quectel_m95.c"


//FUNCTION DECLARATIONS
void reset_char_array(char *array_pointer , uint8_t size);
uint8_t tx_at_response(const m95_at_t *opt);
uint8_t data_to_char(uint16_t *array_data, uint8_t array_data_len, char *array_ascii, int base);
uint16_t controller_calc_avg(uint32_t data, uint16_t cnt);
uint16_t adc_result_average (uint8_t adc_ch, uint8_t num_avg);
static void adc_initialization(void);
void rtc_init_period(uint16_t period);
uint8_t reset_all_data();
void reset_char_array(char *array_pointer , uint8_t size);
uint8_t reset_tx_data(uint16_t *array, uint16_t *reset_array, uint8_t len_array);
void usart_tx_char(USART_t *usart, char *cmd);
uint8_t at_response(USART_t *usart, uint16_t timeout, char *array_pointer);
uint8_t usart_rx_at(USART_t *usart, uint16_t timeout, uint8_t *timeout_status);
void usart_tx_at(USART_t *usart, uint8_t *cmd);
uint8_t radio_power_on(void);
uint8_t tx_data_response(char *data, int len);
uint8_t tx(char *data, int len);
uint8_t radio_power_off_at(void);
uint8_t radio_power_off(void);
void radio_pins_init(void);
void my_delay_10ms(uint8_t loops);
uint8_t loadcell_power_on(void);
void loadcell_pins_init(void);
void led_blink(uint16_t on_time);
uint8_t at_rf_status(void);
uint8_t at_rf_gprs(void);
uint8_t at_rf_disconnect(void);
uint8_t at_rf_connect(uint8_t state);
uint16_t adc_10_to_12_bits (uint8_t adc_ch);

//EXTERNAL SENSOR SETUP
#include "loadcell_logger.h"


//COMMUNICATION PACKAGE SETUP
//COULD BE UPC UA OR OTHER.
#include "MQTT_functions.h"






#include "adc.h"


#endif /* MAIN_H_ */