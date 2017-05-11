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
/*
	TODO:
	- integrate response into tx function?
	- integrate timout 
	- HW data flow
	*/

//status
uint8_t STATUS = 0;

//Functions declarations
void usart_tx_at(USART_t *usart, uint8_t *cmd);
uint8_t at_response(USART_t *usart, uint8_t *cmd);
uint8_t at_response_debug(USART_t *usart, uint8_t *cmd);
void led_blink(uint8_t on_time);

//Functions
void usart_tx_at(USART_t *usart, uint8_t *cmd) {
	
	while(*cmd) {
		usart_putchar(usart, *cmd++);
	}
}

uint8_t at_response(USART_t *usart, uint8_t *cmd) {
	
	uint8_t cmd_len = strlen(cmd);
	uint8_t byte;
	
	char response[20] = "";
	char ret;
	const char ok[sizeof(RESPONSE_OK)-1] = RESPONSE_OK;
	   
	STATUS = 0;
	int i = 0;
	
	while (i < cmd_len+6)
	{
		response[i] = usart_getchar(usart);
		//usart_putchar(USART_SERIAL_EXAMPLE, response[i]);
		i++;
	}
	
	usart_tx_at(USART_SERIAL_EXAMPLE, response);
	//usart_tx_at(USART_SERIAL_EXAMPLE, ok);
	
	strncpy(ret, strstr(response, ok), 2);
	if (strcmp(ret,ok))
	{
		usart_tx_at(USART_SERIAL_EXAMPLE, ret);
		STATUS = 1;
	}
	
	
	return STATUS;
}

uint8_t at_response_debug(USART_t *usart, uint8_t *cmd) {
	
	uint8_t cmd_len = strlen(cmd);
	uint8_t byte;
	
	char response[20] = "";
	STATUS = 0;
	
	int i = 0;
	
	while (i < cmd_len+10)
	{
		usart_putchar(USART_SERIAL_EXAMPLE, response[i]);
		response[i] = usart_getchar(usart);
		i++;
	}
	
	usart_tx_at(USART_SERIAL_EXAMPLE, response);
	
	return STATUS;
}

void led_blink(uint8_t on_time) {
	
	PORTQ.OUT &= ~(1<<3);
	delay_s(on_time);
	PORTQ.OUT |= (1<<3);
	delay_s(on_time);
}


/*! \brief Main function.
 */
int main(void)
{
	
	volatile uint8_t tx_length = sizeof(AT);
	uint8_t received_byte = NULL;
	uint8_t received_byte2;
	uint8_t i;

	
	/* Initialize the board.
	 * The board-specific conf_board.h file contains the configuration of
	 * the board initialization.
	 */
	board_init();
	sysclk_init();
	
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



	
	usart_tx_at(USART_SERIAL_SIM900, AT_CIPSHUT);
	delay_s(1);
	usart_tx_at(USART_SERIAL_SIM900, AT_CIPSTATUS);
	delay_s(1);
	usart_tx_at(USART_SERIAL_SIM900, AT_CIPMUX);
	delay_s(1);
	
	i = 0;
	while (i < LEN_CSTT)
	{
		usart_tx_at(USART_SERIAL_SIM900, AT_CSTT[i]);
		i++;
	}
	delay_s(1);
	
	usart_tx_at(USART_SERIAL_SIM900, AT_CIICR);
	delay_s(3);
	usart_tx_at(USART_SERIAL_SIM900, AT_CIFSR);
	delay_s(1);
	
	i = 0;
	while (i < LEN_CIPSTART)
	{
		usart_tx_at(USART_SERIAL_SIM900, AT_CIPSTART[i]);
		i++;
	}
	delay_s(3);
	
	usart_tx_at(USART_SERIAL_SIM900, AT_CIPSEND);
	delay_s(1);
	usart_tx_at(USART_SERIAL_SIM900, AT_MESSAGE);
	delay_s(1);
	usart_tx_at(USART_SERIAL_SIM900, CTRL_Z);
	delay_s(1);
	usart_tx_at(USART_SERIAL_SIM900, AT_CIPSHUT);
	
	
	/*
	//usart_tx_at(USART_SERIAL_EXAMPLE, AT);
	usart_tx_at(USART_SERIAL_SIM900, AT);
	STATUS = at_response(USART_SERIAL_SIM900, AT);
	if (STATUS)
	{
		led_blink(1);
		STATUS = 0;
	}
	
	//usart_tx_at(USART_SERIAL_EXAMPLE, CR);
	usart_tx_at(USART_SERIAL_SIM900, CR);
	delay_s(1);
	
	
	//usart_tx_at(USART_SERIAL_EXAMPLE, AT_CMGF);
	usart_tx_at(USART_SERIAL_SIM900, AT_CMGF);
	STATUS = at_response(USART_SERIAL_SIM900, AT_CMGF);
	if (STATUS)
	{
		led_blink(1);
		STATUS = 0;
	}
	
	//usart_tx_at(USART_SERIAL_EXAMPLE, AT_CMGS);
	usart_tx_at(USART_SERIAL_SIM900, AT_CMGS);
	//STATUS = at_response(USART_SERIAL_SIM900, AT_CMGS); //not an OK or ERROR response, hence blinking left out.
	delay_s(1);
	led_blink(1);
	
	//usart_tx_at(USART_SERIAL_EXAMPLE, AT_MESSAGE);
	usart_tx_at(USART_SERIAL_SIM900, AT_MESSAGE);
	//STATUS = at_response(USART_SERIAL_SIM900, AT_MESSAGE); //not an OK or ERROR response, hence blinking left out.
	delay_s(1);
	led_blink(1);
	
	//usart_tx_at(USART_SERIAL_EXAMPLE, CTRL_Z);
	usart_tx_at(USART_SERIAL_SIM900, CTRL_Z);
	*/
	
	
	while (true) {
		//nop
	}
}