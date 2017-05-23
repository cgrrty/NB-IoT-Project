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
//#include <tx_gprs.h>
/*
	TODO:
	- integrate response into tx function?
	- integrate timout 
	- HW data flow
	*/




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


typedef enum {
	READ_EXT_DATA,
	MEASURE,
	CALC,
	STORE_EXT_MEM,
	TX_DATA,
	RX_DATA
} controller_states_t;



//status and modes
#define OK 0
#define TIMEOUT 1
#define STATUS_AT_DEBUG "\r\nSTATUS AT: "


volatile uint8_t status_at = 0;
volatile uint8_t status_at_timeout = 0;
volatile uint8_t tx_active = 0;


//timers
#define AT_TIMEOUT_TC TCC0 //define AT command timeout counter.
//#define AT_TIMEOUT 1000



//Functions declarations
void usart_tx_at(USART_t *usart, uint8_t *cmd);
uint8_t usart_rx_at(USART_t *usart);
uint8_t at_response(USART_t *usart);
void led_blink(uint8_t on_time);
int mqtt_packet(char *payload);
void at_timeout_start();
void at_timeout_stop();
void response_debug(uint8_t code);

//Functions
void usart_tx_at(USART_t *usart, uint8_t *cmd) {
	
	while(*cmd) {
		usart_putchar(usart, *cmd++);
	}
}

uint8_t usart_rx_at(USART_t *usart)
{
	at_timeout_start();
	
	while ((usart_rx_is_complete(usart) == false) & (status_at_timeout == 0)) {
	}
	
	at_timeout_stop();
	
	return ((uint8_t)(usart)->DATA);
}

uint8_t at_response(USART_t *usart) {
	
	uint8_t init_done = 0;
	uint8_t len_response = 50;
	char response[len_response];
	uint8_t i = 0;
	uint8_t j = 0;
	status_at_timeout = 0;
	
	while ((status_at_timeout == 0) & (init_done == 0)) //usually the AT command is sent in return followed by \r\n
	{
		response[i] = usart_rx_at(usart);
		
		
		if ((response[i-1] == 0x4f) & (response[i] == 0x4b)) //check if OK is in the response
		{
			init_done = 1;
			i++;
			break;
		}
		
		
		//usart_putchar(USART_SERIAL_EXAMPLE, response[i]);
		i++;
	}
	
	
	
	usart_tx_at(USART_SERIAL_EXAMPLE, CR);
	usart_tx_at(USART_SERIAL_EXAMPLE, LF);
	while (j<i)
	{
		usart_putchar(USART_SERIAL_EXAMPLE, response[j]);
		j++;
	}
	usart_tx_at(USART_SERIAL_EXAMPLE, CR);
	usart_tx_at(USART_SERIAL_EXAMPLE, LF);
	
	
	return status_at_timeout;
}

void led_blink(uint8_t on_time) {
	
	PORTQ.OUT &= ~(1<<3);
	delay_s(on_time);
	PORTQ.OUT |= (1<<3);
	delay_s(on_time);
}

void at_timeout_start() {
	
	AT_TIMEOUT_TC.INTFLAGS |= (1<<0); //clear ovf flag
	AT_TIMEOUT_TC.CNT = 0; //reset counter
	sei(); //enable interrupt
}

void at_timeout_stop() {
	cli(); //disable interrupt
	AT_TIMEOUT_TC.INTFLAGS |= (1<<0); //clear ovf flag
	
}
/////////////MQTT////////////////////
int mqtt_packet(char *payload)
{
	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
	//int rc = 0;
	char buf[200];
	int buflen = sizeof(buf);
	MQTTString topicString = MQTTString_initializer;
	//char* payload = "mypayload";
	int payloadlen = strlen(payload);
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

void response_debug(uint8_t code) {
	usart_tx_at(USART_SERIAL_EXAMPLE, STATUS_AT_DEBUG);
	usart_putchar(USART_SERIAL_EXAMPLE, 0x30+code);
}  

void at_command_timeout_setup() {
	//AT_TIMEOUT_TC.INTCTRLA |= (0<<0); //disable counter
	AT_TIMEOUT_TC.INTCTRLA |= (1<<0);
	AT_TIMEOUT_TC.CTRLA = TC_CLKSEL_DIV1024_gc;
	AT_TIMEOUT_TC.CTRLB |= 0b000;
	AT_TIMEOUT_TC.PER = 6000;
}


ISR(TCC0_OVF_vect) {
	at_timeout_stop();
	status_at_timeout = 1;
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
	//sysclk_init();
	
	//LED setup
	PORTQ.DIR |= (1<<3);
	PORTQ.OUT |= (1<<3);
	
	
		
	// USART for debug (COM port)
	static usart_rs232_options_t USART_SERIAL_OPTIONS = {
		.baudrate = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength = USART_SERIAL_CHAR_LENGTH,
		.paritytype = USART_SERIAL_PARITY,
		.stopbits = USART_SERIAL_STOP_BIT
	};
	
	
	//USART for SIM900
	static usart_rs232_options_t USART_SERIAL_SIM900_OPTIONS = {
		.baudrate = USART_SERIAL_SIM900_BAUDRATE,
		.charlength = USART_SERIAL_SIM900_CHAR_LENGTH,
		.paritytype = USART_SERIAL_SIM900_PARITY,
		.stopbits = USART_SERIAL_SIM900_STOP_BIT
	};

	// Initialize usart driver in RS232 mode
	usart_init_rs232(USART_SERIAL_EXAMPLE, &USART_SERIAL_OPTIONS);
	usart_init_rs232(USART_SERIAL_SIM900, &USART_SERIAL_SIM900_OPTIONS);
	
	
	at_command_timeout_setup();
	//sei();
	
	/*
	//INIT TC
	tc_enable(&AT_TIMEOUT_TC);
//	tc_set_overflow_interrupt_callback(&AT_TIMEOUT_TC, at_command_timeout);
	tc_set_wgm(&AT_TIMEOUT_TC, TC_WG_NORMAL); 
	tc_write_period(&AT_TIMEOUT_TC, 1000); 
	tc_set_overflow_interrupt_level(&AT_TIMEOUT_TC, TC_INT_LVL_LO);
	cpu_irq_enable();  
	tc_write_clock_source(&AT_TIMEOUT_TC, TC_CLKSEL_DIV1024_gc); //sysclk divided.
	///////////////////////////////////////////////
	//tc_disable(&AT_TIMEOUT_TC);
	//at_timeout_stop();
	*/
	
	
	gprs_states_t gprs_state = AT; //CIPSHUT_INIT;
	gprs_states_t gprs_next_state = gprs_state;
	
	
	while(0) {
		//usart_putchar(USART_SERIAL_EXAMPLE, 0x40);
		//delay_s(1);
	}
	
	
	//usart_putchar(USART_SERIAL_EXAMPLE, 0x41);
	tx_active = 1;
	while (tx_active == 1) {
		
		//Configuring the GPRS state machine. Follow specification from flow chart.
		switch(gprs_state) //compare against controller state????
		{
			case AT:
				gprs_next_state = CIPSHUT_INIT;
				//tx_active = 0; //debug
				
				usart_tx_at(USART_SERIAL_SIM900, AT_AT); //return OK
				status_at = at_response(USART_SERIAL_SIM900);
								
				//delay_s(1);
				
				break;
			
			case CIPSHUT_INIT: 
				gprs_next_state = CIPSTATUS;
				//tx_active = 0; //debug
				
				usart_tx_at(USART_SERIAL_SIM900, AT_CIPSHUT); //return OK
				status_at = at_response(USART_SERIAL_SIM900);
				
				//delay_s(1);
				
				break;
			
						
			case CIPSTATUS: 
				gprs_next_state = CIPMUX;
				//tx_active = 0; //debug
				
				usart_tx_at(USART_SERIAL_SIM900, AT_CIPSTATUS); //return OK
				status_at = at_response(USART_SERIAL_SIM900);
				
				//delay_s(1);
				
				break;
			
										
			case CIPMUX: 
				gprs_next_state = CSTT;
				//tx_active = 0; //debug
								
				usart_tx_at(USART_SERIAL_SIM900, AT_CIPMUX); //return OK
				status_at = at_response(USART_SERIAL_SIM900);
				
				//delay_s(1);
				
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
				status_at = at_response(USART_SERIAL_SIM900);
				
				//delay_s(1);
				
				break;
			
						
			case CIICR: 
				gprs_next_state = CIFSR;
				
				usart_tx_at(USART_SERIAL_SIM900, AT_CIICR); //return OK
				status_at = at_response(USART_SERIAL_SIM900);
				
				//delay_s(3);
				
				break;
			
						
			case CIFSR: 
				gprs_next_state = CIPSTART;
				
				usart_tx_at(USART_SERIAL_SIM900, AT_CIFSR); //return IP
				status_at = at_response(USART_SERIAL_SIM900);
				
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
				status_at = at_response(USART_SERIAL_SIM900); //WHY DOES THIS ONE FAIL???????
								
				delay_s(3); //add check for CONNECT before removing this one.
				
				break;
			
						
			case CIPSEND: 
				gprs_next_state = CIPSHUT;
				
				usart_tx_at(USART_SERIAL_SIM900, AT_CIPSEND); //return >
				delay_s(1);
				char* AT_MESSAGE2 = "0 0 0 512 1023 125";
				//usart_tx_at(USART_SERIAL_SIM900, AT_MESSAGE2);
				//void mqtt_connect();
				mqtt_packet(AT_MESSAGE2);
				delay_s(1);
				usart_tx_at(USART_SERIAL_SIM900, CTRL_Z); //return OK
				status_at = at_response(USART_SERIAL_SIM900);
				
				//delay_s(1);
				
				break;
			
			
			case CIPSHUT: 
				//gprs_next_state = CIPSHUT_INIT;
				tx_active = 0; //done for now...
							
				usart_tx_at(USART_SERIAL_SIM900, AT_CIPSHUT); //return OK
				status_at = at_response(USART_SERIAL_SIM900);
				
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
	
	led_blink(1);
	
	//done
	while (1)
	{
		//nop
	}
	
}
