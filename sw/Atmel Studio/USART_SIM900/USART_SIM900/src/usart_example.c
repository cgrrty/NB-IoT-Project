/**
 * \file
 *
 * \brief AVR XMEGA USART example
 *
 * Copyright (C) 2010-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/*! \mainpage
 * \section intro Introduction
 * This example demonstrates how to use XMEGA USART module.
 *
 * \section files Main Files
 * - usart_example.c: the example application.
 * - conf_board.h: board configuration
 * - conf_usart_example.h: configuration of the example
 *
 * \section usart_apiinfo drivers/usart API
 * The USART driver API can be found \ref usart_group "here".
 *
 * \section deviceinfo Device Info
 * All AVR XMEGA devices can be used.
 * This example has been tested with the following setup:
 *   - STK600 
 *     USARTC0 should be connected to the RS232 spare port of STK600
 *       Note:
 *       XMEGA-A1 on STK600-RC100X
 *       XMEGA-A3, XMEGA-A3U and XMEGA-C3 on STK600-RC064X.
 *       XMEGA-A4U on STK600-RC044X
 *       XMEGA-E5 on STK600-RC032X
 *   - Xplain evaluation kit
 *     USARTD0 on PORTD is used by default
 *     Change to USARTC0 to use the USB Virtual COM PORT of the Xplain
 *   - XMEGA A1 Xplained evaluation kit
 *     USARTC0 on PORTC is used. It is located on the J4 header
 *   - XMEGA A3BU Xplained evaluation kit
 *     USARTC0, pin 2 and 3 on header J1 is utilized
 *   - XMEGA E5 Xplained evaluation kit
 *     USARTD0 on PORTD pin 6 and 7 is used (Connected to board controller)
 * UART configuration is 9600 baudrate, no parity, data 8 bit.
 *
 * \section exampledescription Description of the example
 * The example waits for a received character on the configured USART and
 * echoes the character back to the same USART.
 *
 * \section compinfo Compilation Info
 * This software was written for the GNU GCC and IAR for AVR.
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com/avr">Atmel AVR</A>.\n
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <conf_usart_example.h>
#include <asf.h>
#include <string.h>
#include <sim900_at_commands.h>
#include <mqtt/MQTTPacket.h>
#include <avr/interrupt.h>
#include <math.h>
//#include <tx_gprs.h>
/*
	TODO:
	- integrate response into tx function?
	- integrate timout 
	- HW data flow
	*/


//DEFINE DEBUG. REMOVE IN FINAL CODE!
//#define _DEBUG 1

typedef enum {
	PIN6CTRL
} pinctrl_t;

typedef enum {
	AT,
	CIPSHUT_INIT, //reset if any previous IP sessions are not closed.
	CIPSTATUS,
	CIPMUX,
	CSTT,
	CIICR,
	CIFSR,
	CIPSTART,
	CIPSEND,
	CIPSHUT
} gprs_states_t;

/*
typedef enum {
	AT,
	CIPSHUT_INIT, //reset if any previous IP sessions are not closed.
	CIPSTATUS,
	CIPMUX,
	CSTT,
	CIICR,
	CIFSR,
	CIPSTART,
	CIPSEND,
	CIPSHUT
} nbiot_states_t;
*/

typedef enum {
	READ_EXT_DATA,
	MEASURE,
	CALC,
	STORE_EXT_MEM,
	TX_DATA,
	RX_DATA
} controller_states_t;

//reset and status pins
#define PWRKEY_PORT PORTE 
#define PWRKEY_PIN 6
#define STATUS_PORT PORTE
#define STATUS_PIN 7
#define NETLIGHT_PORT PORTR
#define NETLIGHT_PIN 0



//define states before start
controller_states_t controller_state = MEASURE; //CHANGE IN FINAL.

//sampling and storage
static const uint16_t TS = 5; //actual sampling in seconds
static const uint16_t TTX = 20; //actual transfer rate in seconds
static const uint16_t TAVG = 20; //actual averaging time in seconds

//uint16_t accu_data[3600]; //allocating internal accumulation storage.
uint32_t accu_data = 0; //allocating internal accumulation storage.
uint16_t accu_data_cnt = 0; //counter for accu data.
uint16_t avg_data = 0;
#define MIN_DATA_RESET 0xffff
uint16_t min_data = MIN_DATA_RESET;
uint16_t max_data = 0;
uint16_t tran_data = 0;

#define POSITION_PREV 0
#define POSITION_TRAN_PREV 1
#define POSITION_TRAN_MAX 2
#define POSITION_AVG 3
#define POSITION_MIN 4
#define POSITION_MAX 5
//#define POSITION_xxx 6 //reserved.
#define POSITION_ACCU_CNT 7
#define TX_DATA_SIZE 9 //sum of the above +1.
uint16_t tx_data[TX_DATA_SIZE]; //prev, prev_tran, tran_max, avg, min, max => data to be transferred to external memory and over air.

//status and modes
#define OK 0
#define TIMEOUT 1
#define STATUS_AT_DEBUG "\r\nSTATUS AT: "


volatile uint8_t status_at = 0;
volatile uint8_t status_at_timeout = 0;
volatile uint8_t tx_active = 0;
volatile uint8_t controller_active = 0;

//interrupt
#define INT_LEVEL_LOW 0x01

//timers
#define AT_TIMEOUT_TC TCC0 //define AT command timeout counter.
//#define AT_TIMEOUT 1000

#define CONFIG_RTC_PRESCALER RTC_PRESCALER_DIV1024_gc
#define CONFIG_RTC_SOURCE SYSCLK_RTCSRC_ULP

//ADC
#define ADC_LC ADCA //define ADC A for load cell
#define ADC_LC_CH ADC_CH0 //define channel 0 for load cell measurements.

//data variables
	//uint16_t loadcell_adc_result = 0;
// 	uint16_t loadcell_adc_result_prev = 0;
// 	int16_t loadcell_adc_result_tran = 0; //signed due to difference in both directions
// 	int16_t loadcell_adc_result_tran_prev = 0; //signed due to difference in both directions
// 	int16_t loadcell_adc_result_tran_max = 0; //signed due to difference in both directions
// 	
	//int loadcell_adc_result_low;
// 	int loadcell_adc_result_mid;
// 	int loadcell_adc_result_hi;
// 	
	char avg_ascii[5] = "";
	char min_ascii[5] = "";
	char max_ascii[5] = "";
	char tran_ascii[5] = "";
	
	uint8_t year = 0;
	uint8_t month = 0;
	uint8_t day = 0;
	uint8_t hour = 0;
	uint8_t minute = 0;
	uint8_t second = 0;
	char year_ascii[3] = "";
	char month_ascii[3] = "";
	char day_ascii[3] = "";
	char hour_ascii[3] = "";
	char minute_ascii[3] = "";
	char second_ascii[3] = "";
	#define TRANSFER_DATA_SIZE 64
	char transfer_data[TRANSFER_DATA_SIZE] = ""; //currently 45 if sent as text.
////////////////////////////////////////////////////////////////////////////////////

//AT PARSER

////////////////////////////////////////////////////////////////////////////

//Functions declarations
void usart_tx_at(USART_t *usart, uint8_t *cmd);
uint8_t usart_rx_at(USART_t *usart, uint16_t timeout);
uint8_t at_response(USART_t *usart, uint16_t timeout);
void led_blink(uint8_t on_time);
//int mqtt_packet(char *payload);
void at_timeout_start(uint16_t timeout);
void at_timeout_stop();
void response_debug(uint8_t code);

//Functions
void usart_tx_at(USART_t *usart, uint8_t *cmd) {
	
	//send the command
	while(*cmd) {
		usart_putchar(usart, *cmd++);
	}
	
	/*
	usart_tx_at(USART_SERIAL_EXAMPLE, RESPONSE_OK);
	//check the response
	if (at_response(usart)) //if timeout
	{
		//#ifdef _DEBUG == 1
		usart_tx_at(USART_SERIAL_EXAMPLE, RESPONSE_ERROR);
		//#endif // _DEBUG
	}
	*/
	
}

uint8_t usart_rx_at(USART_t *usart, uint16_t timeout)
{
	at_timeout_start(timeout);
	
	while ((usart_rx_is_complete(usart) == false) & (status_at_timeout == 0)) {
	}
	
	at_timeout_stop();
	
	return ((uint8_t)(usart)->DATA);
}

uint8_t at_response(USART_t *usart, uint16_t timeout) {
	
	uint8_t len_response = 50;
	char response[len_response];
	uint8_t i = 0;
	uint8_t j = 0;
	status_at_timeout = 0;
	uint8_t status = 1;
	
	
	while (status_at_timeout == 0) //usually the AT command is sent in return followed by \r\n
	{
		response[i] = usart_rx_at(usart, timeout);
		
		
		if ((response[i-1] == 0x4f) & (response[i] == 0x4b)) //check if OK is in the response
		{
			//status = 0;
		}
		
		
		if ( (response[i-2] == 0x3a & response[i-1] == 0x20) ) //check if ': ' is in the response
		{
			if ((response[i] >= 0x30 & response[i] <= 0x39))
			{
				status = response[i] - 0x30;
			}
		}
		
		//usart_putchar(USART_SERIAL_EXAMPLE, response[i]);
		i++;
	}
	
// 	usart_tx_at(USART_SERIAL_EXAMPLE, CR);
// 	usart_tx_at(USART_SERIAL_EXAMPLE, LF);
		
	while (j<i)
	{
		usart_putchar(USART_SERIAL_EXAMPLE, response[j]);
		//usart_tx_at(USART_SERIAL_EXAMPLE, CR);
		//usart_tx_at(USART_SERIAL_EXAMPLE, LF);
		j++;
	}
// 	usart_tx_at(USART_SERIAL_EXAMPLE, CR);
// 	usart_tx_at(USART_SERIAL_EXAMPLE, LF);
		
	//usart_putchar(USART_SERIAL_EXAMPLE, resp_nr);
	return status; //_at_timeout;
}

uint8_t at_response_num(USART_t *usart, uint16_t timeout) {
	
	uint8_t init_done = 0;
	uint8_t len_response = 50;
	char response[len_response];
	uint8_t i = 0;
	uint8_t j = 0;
	status_at_timeout = 0;
	uint8_t response_num = 0;
	uint8_t response_cnt = 0;
	
	while ((status_at_timeout == 0) & (init_done == 0)) //usually the AT command is sent in return followed by \r\n
	{
		response[i] = usart_rx_at(usart, timeout);
			
		/*	
		if (((response[i-1] >= 0x30) & (response[i-1] <= 0x39)) & (response[i] == 0x0d) & (status_at_timeout == 0)) //CR + LF => text in front
		{
			//usart_putchar(USART_SERIAL_EXAMPLE, 0x40); //@ if text
			response_num = response[i-1];
			break;
		}
		*/
		
		if (((response[i] >= 0x30) & (response[i] <= 0x39)) & (status_at_timeout == 0)) //CR + LF => text in front
		{
			//usart_putchar(USART_SERIAL_EXAMPLE, response[i]);
// 			usart_tx_at(USART_SERIAL_EXAMPLE, CR);
// 			usart_tx_at(USART_SERIAL_EXAMPLE, LF);
			response_num = response[i];
			response_cnt++;
			response_num = response_cnt+0x30;
			//break;
		}
		
		//usart_putchar(USART_SERIAL_EXAMPLE, response[i]);
		//usart_putchar(USART_SERIAL_EXAMPLE, 0x0a);
		
		i++;
		
	}
	
	
	
	
	usart_tx_at(USART_SERIAL_EXAMPLE, CR);
	usart_tx_at(USART_SERIAL_EXAMPLE, LF);
	/*	
	while (j<i)
	{
		if (response[j] == 0x0d) //CR
		{
			if (response[j+1] != 0x0a)
			{
				response_num = response[j-1];
				break;
			}
			//usart_putchar(USART_SERIAL_EXAMPLE, 0x40); //@ if text
		}
		
		//usart_putchar(USART_SERIAL_EXAMPLE, response[j]);
		j++;
	}
	usart_tx_at(USART_SERIAL_EXAMPLE, CR);
	usart_tx_at(USART_SERIAL_EXAMPLE, LF);
	*/
	
	usart_putchar(USART_SERIAL_EXAMPLE, response_num);
	return response_num;
}

void led_blink(uint8_t on_time) {
	
	PORTQ.OUT &= ~(1<<3);
	delay_s(on_time);
	PORTQ.OUT |= (1<<3);
	delay_s(on_time);
}

void rtc_init_period(uint16_t period)
{
	sysclk_enable_module(SYSCLK_PORT_GEN, SYSCLK_RTC);
	RTC.PER = period; //0x0001; //0xffff;
	RTC.CNT = 0;
	/* Since overflow interrupt is needed all the time we limit sleep to
	 * power-save.
	 */
	sleepmgr_lock_mode(SLEEPMGR_PSAVE);
	RTC.INTCTRL |= INT_LEVEL_LOW;
	RTC.CTRL = RTC_PRESCALER_DIV1024_gc;
}


void at_timeout_start(uint16_t timeout) {
	AT_TIMEOUT_TC.PER = timeout; //2000 = 1 second, default ~300ms.
	AT_TIMEOUT_TC.INTFLAGS |= (1<<0); //clear ovf flag
	AT_TIMEOUT_TC.CNT = 0; //reset counter
	sei(); //enable interrupt
}

void at_timeout_stop() {
	cli(); //disable interrupt
	AT_TIMEOUT_TC.INTFLAGS |= (1<<0); //clear ovf flag
	
}
/////////////MQTT////////////////////
int mqtt_packet(char *payload) //*payload is the original
{
	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
	
	char buf[200];
	int buflen = sizeof(buf);
	MQTTString topicString = MQTTString_initializer;
	
	int payloadlen = strlen(payload);
	//int payloadlen = 4;
	int len = 0;
	
	data.clientID.cstring = "SIM900";
	data.keepAliveInterval = 20;
	data.cleansession = 1;
	data.username.cstring = "";
	data.password.cstring = "";
	data.MQTTVersion = 4;

	len = MQTTSerialize_connect((unsigned char *)buf, buflen, &data);

	topicString.cstring = "home/garden/fountain";
	len += MQTTSerialize_publish((unsigned char *)(buf + len), buflen - len, 0, 0, 0, 0, topicString, (unsigned char *)payload, payloadlen);
	
	len += MQTTSerialize_disconnect((unsigned char *)(buf + len), buflen - len);

	int i = 0;
	while (i<len)
	{
		usart_putchar(USART_SERIAL_SIM900, buf[i]);
		i++;
	}
	usart_putchar(USART_SERIAL_SIM900, CR); //end package (SIM900 requirement in send mode)
	
	exit:
	
	return 0;
}

///////////////////////////////////////////////////

void at_command_timeout_setup() {
	//AT_TIMEOUT_TC.INTCTRLA |= (0<<0); //disable counter
	AT_TIMEOUT_TC.INTCTRLA |= (1<<0);
	AT_TIMEOUT_TC.CTRLA = TC_CLKSEL_DIV1024_gc;
	AT_TIMEOUT_TC.CTRLB |= 0b000;
	AT_TIMEOUT_TC.PER = 700; //2000 = 1 second, default ~300ms.
}

static void adc_init(void)
{
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;
	adc_read_configuration(&ADC_LC, &adc_conf);
	adcch_read_configuration(&ADC_LC, ADC_LC_CH, &adcch_conf);
	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12, ADC_REF_VCC); //vdd/1,6 ~ 2V @ 3,3V.
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_set_clock_rate(&adc_conf, 200000UL);
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN0, ADCCH_NEG_NONE, 1);
	//adcch_set_input(&adcch_conf, ADCCH_POS_SCALED_VCC, ADCCH_NEG_NONE, 1);
	adc_write_configuration(&ADC_LC, &adc_conf);
	adcch_write_configuration(&ADC_LC, ADC_LC_CH, &adcch_conf);
}

uint16_t adc_result_average (uint8_t num_avg) {
	
	uint8_t i = 0;
	uint32_t res = 0;
	uint16_t res_median[num_avg];
	
	while (i<num_avg)
	{
		adc_start_conversion(&ADC_LC, ADC_LC_CH);
		adc_wait_for_interrupt_flag(&ADC_LC, ADC_LC_CH);
		res_median[i] = adc_get_result(&ADC_LC, ADC_LC_CH);
		res = res + res_median[i];
		i++;
	}
	
	res = res/num_avg;
	
	
	//return res_median[(num_avg-1)/2];
	return res;	
}

uint16_t controller_measure(uint8_t nr_samples, uint16_t array[TX_DATA_SIZE]) {
	
	//measure with adc
	uint16_t adc_result = 0;
	adc_result = adc_result_average(nr_samples);
	//adc_result = 1234;
	////////////////////////////////////////////////
	
	

	sprintf(transfer_data, "%d", adc_result); //convert to hex to lower transferred bytes.
	
		/*
		strcpy(debug_text, debug_byte);
		strcat(debug_text, ",");
		strcat(debug_text, ",");
		*/
		usart_tx_at(USART_SERIAL_EXAMPLE, transfer_data);
	
	////////////////////////////////////////////////////////////////	
	
	//find tran
	uint16_t tran = 0;	 
	tran = adc_result - array[POSITION_PREV]; //tran = current - previous
	if ((abs(tran) > abs(array[POSITION_TRAN_MAX])) & (array[POSITION_ACCU_CNT] > 0)) //first step is not valid due to only one value.
    {
        array[POSITION_TRAN_MAX] = tran; //store new tran max.
		
    }
	array[POSITION_PREV] = adc_result; //store current adc value as previous for next tran calculation.
	///////////////////////////////////////////////////////////////////////////////////////////////////////
	
    //debug
	/*
    char debugdata[5];
    itoa(loadcell_adc_result, debugdata, 10);
    usart_tx_at(USART_SERIAL_EXAMPLE, debugdata);
    usart_tx_at(USART_SERIAL_EXAMPLE, SPACE);
    */
	
	//find min and max
    if (adc_result < array[POSITION_MIN])
    {
        array[POSITION_MIN] = adc_result; //store new min value.
    }
    if (adc_result > array[POSITION_MAX])
    {
        array[POSITION_MAX] = adc_result; //Store new max.
    }
    
         
	return adc_result; //return current measurement.  
}

uint16_t controller_calc_avg(uint32_t data, uint16_t cnt) {
	 uint16_t avg = 0;
	 
	 avg = data/cnt;
	 return avg;
}

void controller_tx(uint16_t array[TX_DATA_SIZE]) {
	
	//controller_next_state = RX_DATA;
    itoa(array[POSITION_AVG], avg_ascii, 10); //convert to hex to lower transferred bytes.
    itoa(array[POSITION_MIN], min_ascii, 10); //convert to hex to lower transferred bytes.
    itoa(array[POSITION_MAX], max_ascii, 10); //convert to hex to lower transferred bytes.
    itoa(array[POSITION_TRAN_MAX], tran_ascii, 10); //convert to hex to lower transferred bytes.
    
	//reset parameters
	array[0] = 0;
	array[1] = 0;
	array[2] = 0;
	array[3] = 0;
	array[4] = 0;
	array[5] = 0;
           
                 
    year = 17;
    itoa(year, year_ascii,16);
    month = 6;
    itoa(month, month_ascii,16);
    day = 7;
    itoa(day, day_ascii,16);
    hour = 13;
    itoa(hour, hour_ascii,16);
    minute++;
    itoa(minute, minute_ascii, 16);
    second = 0;
    itoa(second, second_ascii,16);
                 
    strcpy(transfer_data, avg_ascii);
    strcat(transfer_data, ",");
    strcat(transfer_data, min_ascii);
    strcat(transfer_data, ",");
    strcat(transfer_data, max_ascii);
    strcat(transfer_data, ",");
    strcat(transfer_data, tran_ascii);
	strcat(transfer_data, ",");
    /*
    strcat(transfer_data, ",");
    strcat(transfer_data, year_ascii);
    strcat(transfer_data, month_ascii);
    strcat(transfer_data, day_ascii);
    strcat(transfer_data, hour_ascii);
    strcat(transfer_data, minute_ascii);
    strcat(transfer_data, second_ascii);
    */

	

    usart_tx_at(USART_SERIAL_EXAMPLE, RESPONSE_HEADER);
	int i=0;
	while(transfer_data[i] != 0x00) {
		usart_putchar(USART_SERIAL_EXAMPLE, transfer_data[i]);
		i++;
	}
    //usart_tx_at(USART_SERIAL_EXAMPLE, transfer_data);
    usart_tx_at(USART_SERIAL_EXAMPLE, RESPONSE_HEADER);
	
}

void reset_tx_data(uint16_t array[TX_DATA_SIZE]) {
	array[POSITION_PREV] = 0;
	array[POSITION_TRAN_PREV] = 0;
	array[POSITION_TRAN_MAX] = 0;
	array[POSITION_AVG] = 0;
	array[POSITION_MIN] = MIN_DATA_RESET;
	array[POSITION_MAX] = 0;
	array[POSITION_ACCU_CNT] = 0;
	
}

void portctrl_setup(PORT_t port, uint8_t pin) {

	switch (pin) {
		case 0: port.PIN0CTRL |= PORT_OPC_PULLDOWN_gc;
		case 1: port.PIN1CTRL |= PORT_OPC_PULLDOWN_gc;
		case 2: port.PIN2CTRL |= PORT_OPC_PULLDOWN_gc;
		case 3: port.PIN3CTRL |= PORT_OPC_PULLDOWN_gc;
		case 4: port.PIN4CTRL |= PORT_OPC_PULLDOWN_gc;
		case 5: port.PIN5CTRL |= PORT_OPC_PULLDOWN_gc;
		case 6: port.PIN6CTRL |= PORT_OPC_PULLDOWN_gc;
		case 7: port.PIN7CTRL |= PORT_OPC_PULLDOWN_gc;
		
	}
}

void radio_pins_init(void) {
	
	//PWRKEY and startup sequence.
	PWRKEY_PORT.DIR |= (1<<PWRKEY_PIN); //reset pin
	PWRKEY_PORT.OUT &= ~(1<<PWRKEY_PIN); //reset
	delay_ms(1); //wait for battery voltage to settle.
	PWRKEY_PORT.OUT |= (1<<PWRKEY_PIN); //reset of radio
	delay_ms(100); //boot time, 100ms recommended for m95
	PWRKEY_PORT.OUT &= ~(1<<PWRKEY_PIN);
	delay_ms(800); //time before m95 is running. There exist a status bit that might be useful to monitor.
	delay_ms(400);
	PWRKEY_PORT.OUT |= (1<<PWRKEY_PIN); //normal level for this pin
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	//STATUS, NOT NEEDED AS NETLIGHT IS REQUIRED BEFORE SENDING COMMANDS.
	STATUS_PORT.DIR &= ~(1<<STATUS_PIN); //input
	//portctrl_setup(STATUS_PORT, STATUS_PIN); //should not be needed.
	
	//NETLIGHT. NOT AVAILABLE ON DEVELOPMENT BOARD, HAVE TO USE SW CALL TO CHECK FOR CONNECTION STATUS.
	//NETLIGHT_PORT.DIR &= ~(1<<NETLIGHT_PIN); //input
	
}

void radio_power_down(void) {
	//power down
	PWRKEY_PORT.OUT &= ~(1<<PWRKEY_PIN);
	delay_s(1);
	PWRKEY_PORT.OUT |= (1<<PWRKEY_PIN); 
	delay_s(1);
	PWRKEY_PORT.OUT &= ~(1<<PWRKEY_PIN);
}

ISR(TCC0_OVF_vect) {
	at_timeout_stop();
	status_at_timeout = 1;
}


ISR(RTC_OVF_vect)
{
	cli(); //disable interrupts. Other way of disabling and resetting?
	//rtc_data.counter_high++;
	
	//led_blink(1);
	
	
	if (controller_state == MEASURE)
	{
		accu_data += controller_measure(9, &tx_data); //measure with averaging, and accumulate.
	}
	
	tx_data[POSITION_ACCU_CNT]++; //increase accumulation counter.
				
	if (tx_data[POSITION_ACCU_CNT] > (TAVG/TS)) //if accumulation limit is reached.
	{
		tx_data[POSITION_AVG] = controller_calc_avg(accu_data, tx_data[POSITION_ACCU_CNT]); //calc and store average.
				
		//reset parameters
		accu_data = 0;
		tx_data[POSITION_ACCU_CNT] = 0;
		
		controller_state = TX_DATA;
		
		//debug
		/*
		itoa(avg_data, loadcell_adc_result_ascii, 10); //convert to hex to lower transferred bytes.
		strcpy(transfer_data, loadcell_adc_result_ascii);
		usart_tx_at(USART_SERIAL_EXAMPLE, RESPONSE_HEADER);
		usart_tx_at(USART_SERIAL_EXAMPLE, transfer_data);
		usart_tx_at(USART_SERIAL_EXAMPLE, RESPONSE_HEADER);
		*/
	}
	
	if (controller_state == TX_DATA)
	{
		controller_tx(&tx_data);
		reset_tx_data(&tx_data);
		
		controller_state = MEASURE;
		
	}
	
	//usart_putchar(USART_SERIAL_EXAMPLE, 0x30+accu_data_cnt);
	//WRITE RTC value = 0!
	RTC.CNT = 0;
	sei(); //enable interrupt, go to sleep
}


/*! \brief Main function.
 */
int main(void)
{
	cli();
	
	//general counter variable used by many functions.
	uint8_t i = 0;
	
		
	/* Initialize the board.
	 * The board-specific conf_board.h file contains the configuration of
	 * the board initialization.
	 */
	board_init();
	//pmic_init(); //needed for TC ASF code. Check if needed in real implementation.
	PMIC.CTRL = 0x01; //low level interrupt
	//sysclk_init(); //fucks up the at_response....AHGHHHHHHHHHHHHH
	
	//select system clock
	//CLK.CTRL = 0x01; //2M
		
	
	
	//LED setup
	PORTQ.DIR |= (1<<3);
	PORTQ.OUT |= (1<<3);
	
	//ADC setup
	adc_init();
	adc_enable(&ADC_LC); //Later??? By interrupt?
		
	// USART for debug (COM port)
	static usart_rs232_options_t USART_SERIAL_OPTIONS = {
		.baudrate = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength = USART_SERIAL_CHAR_LENGTH,
		.paritytype = USART_SERIAL_PARITY,
		.stopbits = USART_SERIAL_STOP_BIT
	};
	
	///////////////////////////////
	

	usart_init_rs232(USART_SERIAL_EXAMPLE, &USART_SERIAL_OPTIONS);
	
	usart_init_rs232(USART_SERIAL_SIM900, &USART_SERIAL_OPTIONS);
	
	at_command_timeout_setup();
	//sei();
	
	//WDT setup
	
	
	//reset data
	reset_tx_data(&tx_data);
	
	//RTC
	/*
	sleepmgr_init();
	rtc_init_period(TS); //using RTC as sampler timer.
	cpu_irq_enable();
	
	
	
	
	
	while (1)
	{
		sleepmgr_enter_sleep();
		//controller_execute_debug();
	}
	*/
	
	
	//DEBUG M95
	sei();
	
	
	PORTQ.OUT &= ~(1<<3); //led on while waiting for status
	radio_pins_init(); //initialize pins towards radio module. Wake up radio.
	
	
	//wait for status
	while (!(STATUS_PORT.IN & (1<<STATUS_PIN)));
	PORTQ.OUT |= (1<<3); //led off
	
	
	
	//AT+CREG??????????
	/*
	#define AT_CREG "AT+CREG?\n"
	uint8_t mystatus = 0;
	while (mystatus == 0)
	{
		usart_tx_at(USART_SERIAL_SIM900, AT_CREG); //return +QNSTATUS: n, where 0 is ok.
		if (at_response_creg(USART_SERIAL_SIM900, RESPONSE_TIME_300M)) {usart_tx_at(USART_SERIAL_EXAMPLE, RESPONSE_OK); mystatus = 1;}
		
		delay_ms(2000);
	}
	*/
	
	//SETTING RESPONSE FORMAT
	#define ATV0 "ATV0\r" //response format is numbers
	#define ATV1 "ATV1\r" //response format is text
	
	uint8_t mychar = 0;
	
	/*
	i=0;
	while (i<2)
	{
	usart_tx_at(USART_SERIAL_SIM900, ATV0); //return +QNSTATUS: n, where 0 is ok.
	mychar = at_response_num(USART_SERIAL_SIM900, RESPONSE_TIME_300M);
	delay_s(1);
	i++;
	}
	*/
	//usart_putchar(USART_SERIAL_EXAMPLE, mychar);
	
		
	usart_tx_at(USART_SERIAL_SIM900, AT_QISTAT); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M);
		
	//Wait for GSM network status
	mychar = 1;
 	while (mychar != 0)
 	{
		usart_tx_at(USART_SERIAL_SIM900, AT_QNSTATUS); //return +QNSTATUS: n, where 0 is ok.
		mychar  = at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M);
		delay_s(1);
	}
	
	//delay_s(2);
	
	//usart_tx_at(USART_SERIAL_SIM900, AT_QICLOSE); //return OK. Perhaps not needed if waking up from power down.
	//if (at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M)) {usart_tx_at(USART_SERIAL_EXAMPLE, RESPONSE_ERROR);}
	//delay_s(1);
	usart_tx_at(USART_SERIAL_SIM900, AT_QISTAT); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M);
		
	
	//delay_s(1);
	usart_tx_at(USART_SERIAL_SIM900, AT_QIFGCNT); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M);
	//delay_s(1);
	usart_tx_at(USART_SERIAL_SIM900, AT_QICSGP); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M);
	//delay_ms(300); //max response time 300ms.
	usart_tx_at(USART_SERIAL_SIM900, AT_QIMUX); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M);
	//delay_s(1);
	usart_tx_at(USART_SERIAL_SIM900, AT_QIMODE); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M);
	//delay_s(1);
	usart_tx_at(USART_SERIAL_SIM900, AT_QIDNSIP); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M);
	//delay_s(1);
	usart_tx_at(USART_SERIAL_SIM900, AT_QIREGAPP); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M);
	//delay_s(1);
	//usart_tx_at(USART_SERIAL_EXAMPLE, AT_QISTAT); //return OK
	usart_tx_at(USART_SERIAL_SIM900, AT_QISTAT); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M);
	
	usart_tx_at(USART_SERIAL_SIM900, AT_QIACT); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_20S);
		
	//delay_s(1);
	usart_tx_at(USART_SERIAL_SIM900, AT_QILOCIP); //return OK //fix response
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M);
	//delay_s(1);
	
	//CHECK IP STATUS
	//mychar = 0;
	//while (mychar != 0x30 || mychar != 0x35 || mychar != 0x37)
	//{
// 		usart_tx_at(USART_SERIAL_SIM900, AT_QISTAT); //return OK
// 		mychar = at_response_num(USART_SERIAL_SIM900, RESPONSE_TIME_300M);
	//}
	
	//usart_tx_at(USART_SERIAL_EXAMPLE, AT_QIOPEN); //return OK
	usart_tx_at(USART_SERIAL_SIM900, AT_QIOPEN); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_20S);
	//at_response_num(USART_SERIAL_SIM900, RESPONSE_TIME_20S);
	//delay_s(1);
	usart_tx_at(USART_SERIAL_SIM900, AT_QISRVC); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M);
	//delay_s(1);
	usart_tx_at(USART_SERIAL_SIM900, AT_QISTAT); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M);

	//delay_s(1);
	usart_tx_at(USART_SERIAL_SIM900, AT_QISTAT); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M);
	
	usart_tx_at(USART_SERIAL_SIM900, AT_QISEND); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M);
	//delay_s(1);
		
	char* AT_MESSAGE2 = "2213 10:33:22";
		
	mqtt_packet(AT_MESSAGE2);
	delay_ms(300);
		
	//delay_s(1);
	usart_tx_at(USART_SERIAL_SIM900, CTRL_Z); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M);
	//delay_s(1);
	//usart_tx_at(USART_SERIAL_SIM900, AT_QISRVC); //return OK REMOVE?????????????
	//if (at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M)) {usart_tx_at(USART_SERIAL_EXAMPLE, RESPONSE_ERROR);}
	//delay_s(1);
	usart_tx_at(USART_SERIAL_SIM900, AT_QICLOSE); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M);
	//delay_s(1);
	usart_tx_at(USART_SERIAL_SIM900, AT_QIDEACT); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_20S);
	//delay_s(1);
	
	
	//radio power down
	radio_power_down();
	
	while (1)
	{
		
		led_blink(1);
		//usart_putchar(USART_SERIAL_EXAMPLE, 0x30);
		//usart_putchar(USART_SERIAL_SIM900, 0x30);
		
		//nop
	}
	//////////////////////////////////////////////////
	
	
	
	//Tx/////////////////////////////////////////////////////////////////////////////////////////
	
	/*
	gprs_states_t gprs_state = AT; //CIPSHUT_INIT;
	gprs_states_t gprs_next_state = gprs_state;
	
	tx_active = 0;
	while (tx_active == 1) {
		
		//Configuring the GPRS state machine. Follow specification from flow chart.
		switch(gprs_state) //compare against controller state????
		{
			case AT:
				gprs_next_state = CIPSHUT_INIT;
				//tx_active = 0; //debug
				
				usart_tx_at(USART_SERIAL_SIM900, AT_AT); //return OK
				
				if (at_response(USART_SERIAL_SIM900)) //if timeout
				{
					usart_tx_at(USART_SERIAL_EXAMPLE, RESPONSE_ERROR);
					tx_active = 0; //stop Tx.
				}
				
				break;
			
			case CIPSHUT_INIT: 
				gprs_next_state = CIPSTATUS;
				//tx_active = 0; //debug
				
				usart_tx_at(USART_SERIAL_SIM900, AT_CIPSHUT); //return OK
				if (at_response(USART_SERIAL_SIM900)) //if timeout
				{
					usart_tx_at(USART_SERIAL_EXAMPLE, RESPONSE_ERROR);
					tx_active = 0; //stop Tx.
				}
				
				break;
			
						
			case CIPSTATUS: 
				gprs_next_state = CIPMUX;
				//tx_active = 0; //debug
				
				usart_tx_at(USART_SERIAL_SIM900, AT_CIPSTATUS); //return OK
				if (at_response(USART_SERIAL_SIM900)) //if timeout
				{
					usart_tx_at(USART_SERIAL_EXAMPLE, RESPONSE_ERROR);
					tx_active = 0; //stop Tx.
				}
				
				break;
			
										
			case CIPMUX: 
				gprs_next_state = CSTT;
				//tx_active = 0; //debug
								
				usart_tx_at(USART_SERIAL_SIM900, AT_CIPMUX); //return OK
				if (at_response(USART_SERIAL_SIM900)) //if timeout
				{
					usart_tx_at(USART_SERIAL_EXAMPLE, RESPONSE_ERROR);
					tx_active = 0; //stop Tx.
				}
				
				break;
			
			
			case CSTT: 
				gprs_next_state = CIICR;
				//tx_active = 0; //debug
				while (i < LEN_CSTT)
				{
					usart_tx_at(USART_SERIAL_SIM900, AT_CSTT[i]); //return OK
					//usart_tx_at(USART_SERIAL_EXAMPLE, AT_CSTT[i]);
					i++;
				}
				if (at_response(USART_SERIAL_SIM900)) //if timeout
				{
					usart_tx_at(USART_SERIAL_EXAMPLE, RESPONSE_ERROR);
					tx_active = 0; //stop Tx.
				}
				
				break;
			
						
			case CIICR: 
				gprs_next_state = CIFSR;
				
				usart_tx_at(USART_SERIAL_SIM900, AT_CIICR); //return OK
				if (at_response(USART_SERIAL_SIM900)) //if timeout
				{
					usart_tx_at(USART_SERIAL_EXAMPLE, RESPONSE_ERROR);
					tx_active = 0; //stop Tx.
				}
				
				break;
			
						
			case CIFSR: 
				gprs_next_state = CIPSTART;
				
				usart_tx_at(USART_SERIAL_SIM900, AT_CIFSR); //return IP
				if (at_response(USART_SERIAL_SIM900)) //if timeout
				{
					usart_tx_at(USART_SERIAL_EXAMPLE, RESPONSE_ERROR);
					//tx_active = 0; //stop Tx.
				}
				delay_s(1); //for safety, could add this response table as well.
				
				break;
			
			
			case CIPSTART: 
				gprs_next_state = CIPSEND;
				//tx_active = 0; //debug
				
				i = 0;
				while (i < LEN_CIPSTART)
				{
					usart_tx_at(USART_SERIAL_SIM900, AT_CIPSTART[i]); //return several OK
					//usart_tx_at(USART_SERIAL_EXAMPLE, AT_CIPSTART[i]);
					i++;
				}
				if (at_response(USART_SERIAL_SIM900)) //if timeout
				{
					usart_tx_at(USART_SERIAL_EXAMPLE, RESPONSE_ERROR);
					//tx_active = 0; //stop Tx.
				}				
				delay_s(3); //for safety, add check for CONNECT before removing this one.
				
				break;
			
						
			case CIPSEND: 
				gprs_next_state = CIPSHUT;
				
				usart_tx_at(USART_SERIAL_SIM900, AT_CIPSEND); //return >
				delay_s(1);
				
				//char* AT_MESSAGE2 = "2213 10:33:22";
				char* AT_MESSAGE2 = transfer_data;
				//char* AT_MESSAGE2 = 0x17;
				
				mqtt_packet(AT_MESSAGE2);
				//mqtt_packet(loadcell_result);
				//usart_putchar(USART_SERIAL_EXAMPLE, 0x31);
				delay_s(1);
				usart_tx_at(USART_SERIAL_SIM900, CTRL_Z); //return OK
				//usart_putchar(USART_SERIAL_EXAMPLE, 0x32);
				if (at_response(USART_SERIAL_SIM900)) //if timeout
				{
					usart_tx_at(USART_SERIAL_EXAMPLE, RESPONSE_ERROR);
					tx_active = 0; //stop Tx.
				}
				//usart_putchar(USART_SERIAL_EXAMPLE, 0x33);
				break;
			
			
			case CIPSHUT: 
				gprs_next_state = CIPSHUT_INIT;
				tx_active = 0; //done for now...
							
				usart_tx_at(USART_SERIAL_SIM900, AT_CIPSHUT); //return OK
				if (at_response(USART_SERIAL_SIM900)) //if timeout
				{
					usart_tx_at(USART_SERIAL_EXAMPLE, RESPONSE_ERROR);
					tx_active = 0; //stop Tx.
				}
				
				break;
			
						
			default:
			
// 			need to figure out which statements/stage to enter if this occurs.
// 			It will be dependent on the error message. 
// 			Go to sleep?
// 			Measure again?
// 			Transmit again?
// 			Other?
			
			gprs_next_state = CIPSHUT_INIT;
			tx_active = 0;
			break;
		}
		
		gprs_state = gprs_next_state;
		
	}
	////////////////////////////////////////////////////////////////////////////////////////////////
	
	led_blink(1);
	
	//done
	while (1)
	{
		//nop
	}
	*/
}
