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


/*
Data shared between the ISR and your main program must be both volatile and global in scope in the C language. 
Without the volatile keyword, the compiler may optimize out accesses to a variable you update in an ISR,
as the C language itself has no concept of different execution threads. Take the following example:
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
#define SAMPLING_TIME 5
#define AVERAGING_TIME 600
volatile static const uint16_t TS = SAMPLING_TIME; //actual sampling in seconds
volatile static const uint16_t TTX = AVERAGING_TIME; //actual transfer rate in seconds
volatile static const uint16_t TAVG = AVERAGING_TIME; //actual averaging time in seconds

//uint16_t accu_data[3600]; //allocating internal accumulation storage.
uint32_t accu_data = 0; //allocating internal accumulation storage.
#define MIN_DATA_RESET 0xffff

#define POSITION_PREV 0
#define POSITION_TRAN_PREV 1
#define POSITION_TRAN_MAX 2
#define POSITION_AVG 3
#define POSITION_MIN 4
#define POSITION_MAX 5
#define POSITION_TIMESTAMP 6
#define POSITION_ACCU_CNT 7
#define POSITION_VDD_XMEGA 8
#define TX_DATA_SIZE 10 //sum of the above +1.
volatile uint16_t tx_data[TX_DATA_SIZE]; //prev, prev_tran, tran_max, avg, min, max => data to be transferred to external memory and over air.

#define POSITION_YEAR 0
#define POSITION_MONTH 1
#define POSITION_DAY 2
#define POSITION_HOUR 3
#define POSITION_MINUTE 4
#define POSITION_SECOND 5
#define TX_DATE_SIZE 7 //sum of the above +1.
volatile uint8_t tx_date[TX_DATE_SIZE]; //year, month, day, hour, minute, second

#define RESPONSE_SIZE 100
volatile char response[RESPONSE_SIZE];

//status and modes
#define STATUS_AT_DEBUG "\r\nSTATUS AT: "


volatile uint8_t status_at_timeout = 0;


//interrupt
#define INT_LEVEL_LOW 0x01

//timers
#define AT_TIMEOUT_TC TCC0 //define AT command timeout counter.
#define AT_REPEAT 9 //number of times to repeat an AT command.

#define CONFIG_RTC_PRESCALER RTC_PRESCALER_DIV1024_gc
#define CONFIG_RTC_SOURCE SYSCLK_RTCSRC_ULP

//ADC
#define ADC_LC ADCA //define ADC A for load cell
#define ADC_LC_CH ADC_CH0 //define channel 0 for load cell measurements.
#define ADC_VDD_XMEGA_CH ADC_CH1 //define channel 1 for xmega vdd measurements.

char avg_ascii[5] = "";
char min_ascii[5] = "";
char max_ascii[5] = "";
char tran_ascii[5] = "";
	
char timestamp_ascii[5] = "";
char vdd_xmega_ascii[5] = "";
	
char year_ascii[3] = "";
char month_ascii[3] = "";
char day_ascii[3] = "";
char hour_ascii[3] = "";
char minute_ascii[3] = "";
char second_ascii[3] = "";

char vbat_ascii[6] = "";

	
#define TRANSFER_DATA_SIZE 64
char transfer_data[TRANSFER_DATA_SIZE] = ""; //currently 45 if sent as text.
////////////////////////////////////////////////////////////////////////////////////

//AT PARSER

////////////////////////////////////////////////////////////////////////////

//Functions declarations
void usart_tx_at(USART_t *usart, uint8_t *cmd);
uint8_t usart_rx_at(USART_t *usart, uint16_t timeout);
//uint8_t at_response(USART_t *usart, uint16_t timeout, char array[100]);
void led_blink(uint16_t on_time);
//int mqtt_packet(char *payload);
void at_timeout_start(uint16_t timeout);
void at_timeout_stop();
void response_debug(uint8_t code);
uint8_t tx(char data[]);



//Functions
void usart_tx_at(USART_t *usart, uint8_t *cmd) {
	
	//send the command
	while(*cmd) {
		usart_putchar(usart, *cmd++);
	}
	
}

uint8_t usart_rx_at(USART_t *usart, uint16_t timeout)
{
	//at_timeout_start(timeout);
	uint32_t timeout2 = timeout*1000; //300ms
	
	while ((usart_rx_is_complete(usart) == false) & (timeout2 > 0)) {
		timeout2--;
	}
	
	if (timeout2 == 0)
	{
		//usart_tx_at(USART_SERIAL_EXAMPLE, 0x40);
		status_at_timeout = 1;
	}
	//at_timeout_stop();
	
	return ((uint8_t)(usart)->DATA);
}


void at_response(USART_t *usart, uint16_t timeout, char *array_pointer) {
	
	uint8_t i = 0;
	status_at_timeout = 0;
	uint8_t status = 1;
	
	while (status_at_timeout == 0 & i < RESPONSE_SIZE) //usually the AT command is sent in return followed by \r\n
	{
		*(array_pointer+i) = usart_rx_at(usart, timeout);
		i++;
	}
	
	return status;
}


void led_blink(uint16_t on_time) {
	
	PORTQ.OUT &= ~(1<<3);
	delay_ms(on_time);
	PORTQ.OUT |= (1<<3);
	delay_ms(on_time);
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
	//PR.PRPC &= ~(1<<0); //TCC0
	//AT_TIMEOUT_TC.INTCTRLA |= (0<<0); //disable counter
	AT_TIMEOUT_TC.INTCTRLA |= (1<<0);
	AT_TIMEOUT_TC.CTRLA = TC_CLKSEL_DIV1024_gc;
	AT_TIMEOUT_TC.CTRLB |= 0b000;
	AT_TIMEOUT_TC.PER = 700; //2000 = 1 second, default ~300ms.
	AT_TIMEOUT_TC.INTFLAGS |= (1<<0); //clear ovf flag
	AT_TIMEOUT_TC.CNT = 0; //reset counter
}

static void adc_init(void)
{
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;
	struct adc_channel_config adcch_vdd_xmega_conf;
	
	adc_read_configuration(&ADC_LC, &adc_conf);
	
	adcch_read_configuration(&ADC_LC, ADC_LC_CH, &adcch_conf);
	adcch_read_configuration(&ADC_LC, ADC_VDD_XMEGA_CH, &adcch_vdd_xmega_conf);
	
	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12, ADC_REF_VCC); //vdd/1,6 ~ 2V @ 3,3V.
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_set_clock_rate(&adc_conf, 200000UL);
	
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN0, ADCCH_NEG_NONE, 1);
	adcch_set_input(&adcch_vdd_xmega_conf, ADCCH_POS_PIN1, ADCCH_NEG_NONE, 1);
	
	adc_write_configuration(&ADC_LC, &adc_conf);
	
	adcch_write_configuration(&ADC_LC, ADC_LC_CH, &adcch_conf);
	adcch_write_configuration(&ADC_LC, ADC_VDD_XMEGA_CH, &adcch_vdd_xmega_conf);
}

uint16_t adc_result_single(ADC_t *adc, uint8_t ch_mask) {
	adc_start_conversion(adc, ch_mask);
	adc_wait_for_interrupt_flag(adc, ch_mask);
	return adc_get_result(adc, ch_mask);
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

uint16_t controller_measure(uint8_t nr_samples, uint16_t *array) {
	
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
	tran = adc_result - *(array+POSITION_PREV); //tran = current - previous
	if ((abs(tran) > abs(*(array+POSITION_TRAN_MAX))) & (*(array+POSITION_ACCU_CNT) > 0)) //first step is not valid due to only one value.
    {
        *(array+POSITION_TRAN_MAX) = tran; //store new tran max.
		
    }
	*(array+POSITION_PREV) = adc_result; //store current adc value as previous for next tran calculation.
	///////////////////////////////////////////////////////////////////////////////////////////////////////
	
    //debug
	/*
    char debugdata[5];
    itoa(loadcell_adc_result, debugdata, 10);
    usart_tx_at(USART_SERIAL_EXAMPLE, debugdata);
    usart_tx_at(USART_SERIAL_EXAMPLE, SPACE);
    */
	
	//find min and max
    if (adc_result < *(array+POSITION_MIN))
    {
        *(array+POSITION_MIN) = adc_result; //store new min value.
    }
    if (adc_result > *(array+POSITION_MAX))
    {
        *(array+POSITION_MAX) = adc_result; //Store new max.
    }
    
         
	return adc_result; //return current measurement.  
}

uint16_t controller_calc_avg(uint32_t data, uint16_t cnt) {
	 uint16_t avg = 0;
	 
	 avg = data/cnt;
	 return avg;
}

//void controller_tx(uint16_t array[TX_DATA_SIZE]) {
void controller_tx(uint16_t *array_data, uint8_t *array_date) {
	
	uint8_t status = 0;
	
	//controller_next_state = RX_DATA;
    itoa(*(array_data+POSITION_AVG), avg_ascii, 10); //convert to hex to lower transferred bytes.
    itoa(*(array_data+POSITION_MIN), min_ascii, 10); //convert to hex to lower transferred bytes.
    itoa(*(array_data+POSITION_MAX), max_ascii, 10); //convert to hex to lower transferred bytes.
    itoa(*(array_data+POSITION_TRAN_MAX), tran_ascii, 10); //convert to hex to lower transferred bytes.
	itoa(*(array_data+POSITION_TIMESTAMP), timestamp_ascii, 10); //convert to hex to lower transferred bytes.
	itoa(*(array_data+POSITION_VDD_XMEGA), vdd_xmega_ascii, 10); //convert to hex to lower transferred bytes.
    
	/*
    itoa(*(array_date+POSITION_YEAR), year_ascii,10); //convert to hex to lower transferred bytes.
    itoa(*(array_date+POSITION_MONTH), month_ascii,10); //convert to hex to lower transferred bytes.
	itoa(*(array_date+POSITION_DAY), day_ascii,10); //convert to hex to lower transferred bytes.
	itoa(*(array_date+POSITION_HOUR), hour_ascii,10); //convert to hex to lower transferred bytes.
	itoa(*(array_date+POSITION_MINUTE), minute_ascii,10); //convert to hex to lower transferred bytes.
	itoa(*(array_date+POSITION_SECOND), second_ascii,10); //convert to hex to lower transferred bytes.
	*/           
    strcpy(transfer_data, avg_ascii);
    strcat(transfer_data, ",");
    strcat(transfer_data, min_ascii);
    strcat(transfer_data, ",");
    strcat(transfer_data, max_ascii);
    strcat(transfer_data, ",");
    strcat(transfer_data, tran_ascii);
	strcat(transfer_data, ",");
	strcat(transfer_data, timestamp_ascii);
	strcat(transfer_data, ",");
	strcat(transfer_data, vdd_xmega_ascii);
	
    usart_tx_at(USART_SERIAL_EXAMPLE, RESPONSE_HEADER);
	int i=0;
	while(transfer_data[i] != 0x00) {
		usart_putchar(USART_SERIAL_EXAMPLE, transfer_data[i]);
		i++;
	}
	
	status = tx(&transfer_data);
    usart_tx_at(USART_SERIAL_EXAMPLE, TX_STATEMENT);
	usart_putchar(USART_SERIAL_EXAMPLE, status+0x30);
    usart_tx_at(USART_SERIAL_EXAMPLE, RESPONSE_HEADER);
	
}

void reset_tx_data(uint16_t *array) {
	*(array+POSITION_PREV) = 0;
	*(array+POSITION_TRAN_PREV) = 0;
	*(array+POSITION_TRAN_MAX) = 0;
	*(array+POSITION_AVG) = 0;
	*(array+POSITION_MIN) = MIN_DATA_RESET;
	*(array+POSITION_MAX) = 0;
	*(array+POSITION_TIMESTAMP) = 0;
	*(array+POSITION_ACCU_CNT) = 0;
	
}

void reset_tx_date(uint8_t *array) {
	*(array+POSITION_YEAR) = 0;
	*(array+POSITION_MONTH) = 1;
	*(array+POSITION_DAY) = 1;
	*(array+POSITION_HOUR) = 0;
	*(array+POSITION_MINUTE) = 0;
	*(array+POSITION_SECOND) = 0;
	
	
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
	
	
	//STATUS, NOT NEEDED AS NETLIGHT IS REQUIRED BEFORE SENDING COMMANDS.
	STATUS_PORT.DIR &= ~(1<<STATUS_PIN); //input
	//portctrl_setup(STATUS_PORT, STATUS_PIN); //should not be needed.
	
	//NETLIGHT. NOT AVAILABLE ON DEVELOPMENT BOARD, HAVE TO USE SW CALL TO CHECK FOR CONNECTION STATUS.
	//NETLIGHT_PORT.DIR &= ~(1<<NETLIGHT_PIN); //input
	
}

void radio_power_on(void) {
	PWRKEY_PORT.OUT &= ~(1<<PWRKEY_PIN); //reset
	delay_ms(1); //wait for battery voltage to settle.
	PWRKEY_PORT.OUT |= (1<<PWRKEY_PIN); //reset of radio
	delay_ms(100); //boot time, 100ms recommended for m95
	PWRKEY_PORT.OUT &= ~(1<<PWRKEY_PIN);
	delay_ms(800); //time before m95 is running. There exist a status bit that might be useful to monitor.
	delay_ms(400);
	PWRKEY_PORT.OUT |= (1<<PWRKEY_PIN); //normal level for this pin
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
}

void radio_power_down(void) {
	//power down
	PWRKEY_PORT.OUT &= ~(1<<PWRKEY_PIN);
	delay_s(1);
	PWRKEY_PORT.OUT |= (1<<PWRKEY_PIN); 
	delay_s(1);
	PWRKEY_PORT.OUT &= ~(1<<PWRKEY_PIN);
}

void reset_char_array(char *array_pointer , uint8_t size) {
	uint8_t i = 0;
	while (i < size)
	{
		*(array_pointer+i) = 0x00;
		i++;
	}
}


//uint8_t tx(char data[TX_DATA_SIZE]) {
uint8_t tx(char *data) {
	
	uint8_t status = 0; //tx status, 0 = alles ok.
	uint8_t tx_at_cnt = 0; //nr of AT command sent
	char *ret; //response pointer
	
	//AT+CREG??????????
	
	//SETTING RESPONSE FORMAT
// 	#define ATV0 "ATV0\r" //response format is numbers
// 	#define ATV1 "ATV1\r" //response format is text
	
	
	ret = 0;
	while ( (ret == 0) & (tx_at_cnt < AT_REPEAT)) //9*300ms ~ 3s
	{
		reset_char_array(&response, RESPONSE_SIZE);
		usart_tx_at(USART_SERIAL_SIM900, AT_QNSTATUS); //return +QNSTATUS: n, where 0 is ok.
		at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M, &response);
		ret = strstr(response, "QNSTATUS: 0");
		delay_ms(1000);
		tx_at_cnt++;
	}
	if (tx_at_cnt == AT_REPEAT)
	{
		return status = 1;
	}
	usart_tx_at(USART_SERIAL_EXAMPLE, response); //DEBUG
	
	reset_char_array(&response, RESPONSE_SIZE);
	usart_tx_at(USART_SERIAL_SIM900, AT_QIFGCNT); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M, &response);
	if (!strstr(response, "OK"))
	{
		return status = 2;
	}
	usart_tx_at(USART_SERIAL_EXAMPLE, response);
	
	reset_char_array(&response, RESPONSE_SIZE);
	usart_tx_at(USART_SERIAL_SIM900, AT_QICSGP); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M, &response);
	if (!strstr(response, "OK"))
	{
		return status = 3;
	}
	usart_tx_at(USART_SERIAL_EXAMPLE, response);
	
	reset_char_array(&response, RESPONSE_SIZE);
	usart_tx_at(USART_SERIAL_SIM900, AT_QIMUX); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M, &response);
	if (!strstr(response, "OK"))
	{
		return status = 4;
	}
	usart_tx_at(USART_SERIAL_EXAMPLE, response);
	
	reset_char_array(&response, RESPONSE_SIZE);
	usart_tx_at(USART_SERIAL_SIM900, AT_QIMODE); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M, &response);
	if (!strstr(response, "OK"))
	{
		return status = 5;
	}
	usart_tx_at(USART_SERIAL_EXAMPLE, response);
	
	reset_char_array(&response, RESPONSE_SIZE);
	usart_tx_at(USART_SERIAL_SIM900, AT_QIDNSIP); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M, &response);
	if (!strstr(response, "OK"))
	{
		return status = 6;
	}
	usart_tx_at(USART_SERIAL_EXAMPLE, response);
	
	reset_char_array(&response, RESPONSE_SIZE);
	usart_tx_at(USART_SERIAL_SIM900, AT_QIREGAPP); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M, &response);
	if (!strstr(response, "OK"))
	{
		return status = 7;
	}
	usart_tx_at(USART_SERIAL_EXAMPLE, response);
	
	//CHECK IP STATUS
	ret = 0;
	tx_at_cnt = 0;
	while ( (ret == 0) & (tx_at_cnt < AT_REPEAT)) //9*300ms ~ 3s
	{
		reset_char_array(&response, RESPONSE_SIZE);
		usart_tx_at(USART_SERIAL_SIM900, AT_QISTAT); //return OK
		//tx_status = at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M);
		at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M, &response);
		ret = strstr(response, "IP START");
		delay_ms(300);
		tx_at_cnt++;
	}
	if (tx_at_cnt == AT_REPEAT)
	{
		return status = 8;
	}
	usart_tx_at(USART_SERIAL_EXAMPLE, response);

	reset_char_array(&response, RESPONSE_SIZE);
	usart_tx_at(USART_SERIAL_SIM900, AT_QIACT); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_20S, &response);
	if (!strstr(response, "QIACT"))
	{
		usart_tx_at(USART_SERIAL_EXAMPLE, response);
		return status = 9;
		
	}
	usart_tx_at(USART_SERIAL_EXAMPLE, response);
	
	//Need local IP
	ret = 0;
	tx_at_cnt = 0;
	while ( (ret == 0) & (tx_at_cnt < AT_REPEAT)) //9*300ms ~ 3s
	{
		reset_char_array(&response, RESPONSE_SIZE);
		usart_tx_at(USART_SERIAL_SIM900, AT_QILOCIP); //return OK //fix response
		at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M, &response);
		ret = strstr(response, LOCAL_IP);
		delay_ms(300);
		tx_at_cnt++;
	}
	if (tx_at_cnt == AT_REPEAT)
	{
		return status = 10;
	}
	usart_tx_at(USART_SERIAL_EXAMPLE, response);
		
	//CHECK IP STATUS
	ret = 0;
	tx_at_cnt = 0;
	while ( (ret == 0) & (tx_at_cnt < AT_REPEAT)) //9*300ms ~ 3s
	{
		reset_char_array(&response, RESPONSE_SIZE);
		usart_tx_at(USART_SERIAL_SIM900, AT_QISTAT); //return OK
		//at_response_qnstatus(USART_SERIAL_SIM900, RESPONSE_TIME_300M);
		at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M, &response);
		ret = strstr(response, "CONNECT OK") || strstr(response, "IP INITIAL") || strstr(response, "IP STATUS") || strstr(response, "IP CLOSE");
		delay_ms(300);
		tx_at_cnt++;
	}
	if (tx_at_cnt == AT_REPEAT)
	{
		return status = 11;
	}
	usart_tx_at(USART_SERIAL_EXAMPLE, response);

	reset_char_array(&response, RESPONSE_SIZE);
	usart_tx_at(USART_SERIAL_SIM900, AT_QIOPEN); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_20S, &response);
	if (!strstr(response, "OK"))
	{
		return status = 12;
	}
	usart_tx_at(USART_SERIAL_EXAMPLE, response);
	
	reset_char_array(&response, RESPONSE_SIZE);
	usart_tx_at(USART_SERIAL_SIM900, AT_QISRVC); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M, &response);
	if (!strstr(response, "OK"))
	{
		return status = 13;
	}
	usart_tx_at(USART_SERIAL_EXAMPLE, response);
	
	//CHECK IP STATUS
	ret = 0;
	tx_at_cnt = 0;
	while ( (ret == 0) & (tx_at_cnt < AT_REPEAT)) //9*300ms ~ 3s
	{
		reset_char_array(&response, RESPONSE_SIZE);
		usart_tx_at(USART_SERIAL_SIM900, AT_QISTAT); //return OK
		at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M, &response);
		ret = strstr(response, "CONNECT OK");
		delay_ms(300);
		tx_at_cnt++;
	}
	if (tx_at_cnt == AT_REPEAT)
	{
		return status = 14;
	}
	usart_tx_at(USART_SERIAL_EXAMPLE, response);
	
	//FETCH TIME FROM NETWORK HERE!!!!
	//IF NOT SEND TIMESTAMP
	ret = 0;
	tx_at_cnt = 0;
	while ( (tx_at_cnt < AT_REPEAT) & (*(ret+20) == 0x00) ) //9*300ms ~ 3s
	{
		reset_char_array(&response, RESPONSE_SIZE);
		usart_tx_at(USART_SERIAL_SIM900, AT_QLTS); //return +QNSTATUS: n, where 0 is ok.
		at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M, &response);
		ret = strstr(response, "QLTS: ");
		delay_ms(300);
		tx_at_cnt++;
	}
	if (tx_at_cnt == AT_REPEAT)
	{
		return status = 15;
	}
	usart_tx_at(USART_SERIAL_EXAMPLE, response); //DEBUG
	//Store time and date in registers, and also append to tx data.
	year_ascii[0] = *(ret+7);
	year_ascii[1] = *(ret+8);
	strcat(data, ",");
	strcat(data, year_ascii);
	tx_date[POSITION_YEAR] = atoi(year_ascii);
	month_ascii[0] = *(ret+10);
	month_ascii[1] = *(ret+11);
	strcat(data, ",");
	strcat(data, month_ascii);
	tx_date[POSITION_MONTH] = atoi(month_ascii);
	day_ascii[0] = *(ret+13);
	day_ascii[1] = *(ret+14);
	strcat(data, ",");
	strcat(data, day_ascii);
	tx_date[POSITION_DAY] = atoi(day_ascii);
	hour_ascii[0] = *(ret+16);
	hour_ascii[1] = *(ret+17);
	strcat(data, ",");
	strcat(data, hour_ascii);
	tx_date[POSITION_HOUR] = atoi(hour_ascii);
	minute_ascii[0] = *(ret+19);
	minute_ascii[1] = *(ret+20);
	strcat(data, ",");
	strcat(data, minute_ascii);
	tx_date[POSITION_MINUTE] = atoi(minute_ascii);
	second_ascii[0] = *(ret+22);
	second_ascii[1] = *(ret+23);
	strcat(data, ",");
	strcat(data, second_ascii);
	tx_date[POSITION_SECOND] = atoi(second_ascii);
	usart_tx_at(USART_SERIAL_EXAMPLE, data); //DEBUG
	///////////////////////////////////////////////////////////////////////////////////////

	//COMMENT OUT IF SENDING TO SERVER IS ENABLED
	return status = 32;
	///////////////////////////////////////
	
	reset_char_array(&response, RESPONSE_SIZE);
	usart_tx_at(USART_SERIAL_SIM900, AT_QISEND); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M, &response);
	if (!strstr(response, ">"))
	{
		return status = 16;
	}
	usart_tx_at(USART_SERIAL_EXAMPLE, response);
			
	char* AT_MESSAGE2 = data;
		
	mqtt_packet(AT_MESSAGE2);
	delay_ms(100);
	
	reset_char_array(&response, RESPONSE_SIZE);
	usart_tx_at(USART_SERIAL_SIM900, CTRL_Z); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M, &response);
	if (!strstr(response, "OK"))
	{
		return status = 17;
	}
	usart_tx_at(USART_SERIAL_EXAMPLE, response);
	
	reset_char_array(&response, RESPONSE_SIZE);
	usart_tx_at(USART_SERIAL_SIM900, AT_QICLOSE); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M, &response);
	if (!strstr(response, "OK"))
	{
		return status = 18;
	}
	usart_tx_at(USART_SERIAL_EXAMPLE, response);
	
	reset_char_array(&response, RESPONSE_SIZE);
	usart_tx_at(USART_SERIAL_SIM900, AT_QIDEACT); //return OK
	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_20S, &response);
	if (!strstr(response, "OK"))
	{
		usart_tx_at(USART_SERIAL_EXAMPLE, response);
		return status = 19;
	}
	usart_tx_at(USART_SERIAL_EXAMPLE, response);
	
	return status;
}

/*
ISR(TCC0_OVF_vect) {
	at_timeout_stop();
	status_at_timeout = 1;
}
*/

ISR(RTC_OVF_vect)
{
	cli(); //disable interrupts. Other way of disabling and resetting?
		
	led_blink(50); //DEBUG
	
	
	if (controller_state == MEASURE)
	{
		accu_data += controller_measure(9, &tx_data); //measure with averaging, and accumulate.
		tx_data[POSITION_VDD_XMEGA] = adc_result_single(&ADC_LC, ADC_VDD_XMEGA_CH);
	}
	
	tx_data[POSITION_TIMESTAMP]++; //increase timestamp counter.
	tx_data[POSITION_ACCU_CNT]++; //increase accumulation counter.
				
	if (tx_data[POSITION_ACCU_CNT] > (TAVG/TS)) //if accumulation limit is reached.
	{
		tx_data[POSITION_AVG] = controller_calc_avg(accu_data, tx_data[POSITION_ACCU_CNT]); //calc and store average.
				
		//reset parameters
		accu_data = 0;
		tx_data[POSITION_ACCU_CNT] = 0;
		
		controller_state = TX_DATA;

	}
	
	if (controller_state == TX_DATA)
	{
		
		//Startup of radio
		radio_power_on();
		
		//wait for status
		while (!(STATUS_PORT.IN & (1<<STATUS_PIN)));
		PORTQ.OUT |= (1<<3); //led off
		////////////////////////////////////////////////////////
		
		
		
		controller_tx(&tx_data, &tx_date);
		reset_tx_data(&tx_data);

		//radio power down
		radio_power_down();
		
		controller_state = MEASURE;
		
	}
	
	RTC.CNT = 0;
	sei(); //enable interrupt, go to sleep
}


/*! \brief Main function.
 */
int main(void)
{
	cli();
	
	//general counter variable used by many functions.
	//uint8_t i = 0;
	
		
	/* Initialize the board.
	 * The board-specific conf_board.h file contains the configuration of
	 * the board initialization.
	 */
	board_init();
	pmic_init(); //needed for TC ASF code. Check if needed in real implementation.
	//PMIC.CTRL = 0x01; //low level interrupt
	sysclk_init(); //fucks up the at_response....AHGHHHHHHHHHHHHH
	
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
	sysclk_enable_module(SYSCLK_PORT_C, 4);
	sysclk_enable_module(SYSCLK_PORT_E, 4);
	//sysclk_enable_peripheral_clock(USARTC0);
// 	
	//uint32_t FCPU = sysclk_get_main_hz();
	//PR.PRPC &= ~(1<<4); //enable the clock
	//PR.PRPE &= ~(1<<4); //enable the clock
	
	
	//Shut down radio if already awake
	radio_pins_init();
	delay_s(1);
	uint8_t cnt_pwrdwn = 0;
	//check if status is off
	if (STATUS_PORT.IN & (1<<STATUS_PIN))
	{
		radio_power_down();
		while ((STATUS_PORT.IN & (1<<STATUS_PIN)) & cnt_pwrdwn < 27)
		{
			delay_ms(300);
			cnt_pwrdwn++;
		}
		
		if (cnt_pwrdwn == 27)
		{
			#define AT_QPOWD "AT+QPOED\r"
			reset_char_array(&response, RESPONSE_SIZE);
			usart_tx_at(USART_SERIAL_SIM900, AT_QPOWD); //return OK
			at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M, &response);
			usart_tx_at(USART_SERIAL_EXAMPLE, response);
		}
	}
	 	
	////////////////////////////////////////////////////////
		
	
	sei();
	
	//WDT setup for interrupt
	
	
	//reset data
	reset_tx_data(&tx_data);
	reset_tx_date(&tx_date);
	
	//RTC
	PR.PRGEN &= ~(1<<2); //enable the RTC clock
	sleepmgr_init();
	rtc_init_period(TS); //using RTC as sampler timer.
	

	
	
	
	
	//DEBUG
// 	usart_tx_at(USART_SERIAL_SIM900, AT_QISTAT); //return OK
// 	at_response(USART_SERIAL_SIM900, RESPONSE_TIME_300M);
// 	usart_putchar(USART_SERIAL_EXAMPLE, 0x40);
	
	while (1)
	{
		//usart_tx_at(USART_SERIAL_EXAMPLE, ENTER_SLEEP);
		sleepmgr_enter_sleep();
		//controller_execute_debug();
	}
		
}
