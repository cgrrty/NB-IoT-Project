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
/*
	TODO:
	- integrate response into tx function?
	- integrate timout 
	- HW data flow
	*/




typedef enum {
	CIPSHUT,
	CIPSTATUS
	} gprs_states_t;
	


//status
uint8_t STATUS = 0;

//Functions declarations
void usart_tx_at(USART_t *usart, uint8_t *cmd);
uint8_t at_response(USART_t *usart);
uint8_t at_response_debug(USART_t *usart, uint8_t *cmd);
void led_blink(uint8_t on_time);

//Functions
void usart_tx_at(USART_t *usart, uint8_t *cmd) {
	
	while(*cmd) {
		usart_putchar(usart, *cmd++);
	}
}

uint8_t at_response(USART_t *usart) {
	
	uint8_t init_done = 0;
	char response[20] = "";
	STATUS = 0;
	uint8_t i = 0;
	uint8_t j = 0;
	
	while (!init_done) //usually the AT command is sent in return followed by \r\n
	{
		response[i] = usart_getchar(usart);
		//if ((response[i-1] == 0x0d) & (response[i] == 0x0a)) //check if \r\n is in the response
		if ((response[i-1] == CR) & (response[i] == LF)) //check if \r\n is in the response
		{
			init_done = 1;
			break;
		}
		//usart_putchar(USART_SERIAL_EXAMPLE, response[i]);
		i++;
	}
	
	j = i;
	while (i < j+4) //after the initial AT return OK or ERROR is sent.
	{
		response[i] = usart_getchar(usart);
		if ((response[i-1] == 0x4f) & (response[i] == 0x4b)) //check if ok is in the response
		{
			STATUS = 1;
			break;
 		}
		//usart_putchar(USART_SERIAL_EXAMPLE, response[i]);
		i++;
	}
	
	
	usart_tx_at(USART_SERIAL_EXAMPLE, CR);
	usart_tx_at(USART_SERIAL_EXAMPLE, LF);
	usart_tx_at(USART_SERIAL_EXAMPLE, response);
	usart_tx_at(USART_SERIAL_EXAMPLE, CR);
	usart_tx_at(USART_SERIAL_EXAMPLE, LF);
	
	return STATUS;
}

void led_blink(uint8_t on_time) {
	
	PORTQ.OUT &= ~(1<<3);
	delay_s(on_time);
	PORTQ.OUT |= (1<<3);
	delay_s(on_time);
}

/////////////MQTT////////////////////
int mqtt_packet(int argc, char *argv[])
{
	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
	int rc = 0;
	char buf[200];
	int buflen = sizeof(buf);
	int mysock = 0;
	MQTTString topicString = MQTTString_initializer;
	char* payload = "mypayload";
	int payloadlen = strlen(payload);
	int len = 0;
	char *host = "m2m.eclipse.org";
	int port = 1883;

	if (argc > 1)
	host = argv[1];

	if (argc > 2)
	port = atoi(argv[2]);

	//mysock = transport_open(host,port);
	if(mysock < 0)
	return mysock;

	printf("Sending to hostname %s port %d\n", host, port);

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

	//rc = transport_sendPacketBuffer(mysock, buf, len);
	int i = 0;
	while (i<len)
	{
		usart_putchar(USART_SERIAL_SIM900, buf[i]);
		i++;
	}
	usart_putchar(USART_SERIAL_SIM900, CR); //end package (SIM900 requirement in send mode)
	
	
	if (rc == len)
	printf("Successfully published\n");
	else
	printf("Publish failed\n");

	exit:
	//transport_close(mysock);
	return 0;
}

///////////////////////////////////////////////////


/*! \brief Main function.
 */
int main(void)
{
	
	
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
	
	
	/*MQTT*/
	int status = 0; 
	//status = mqtt_packet(1, "10.8.10.136");	
	//usart_putchar(USART_SERIAL_EXAMPLE, status+0x30);
	

	

	//usart_tx_at(USART_SERIAL_SIM900, AT);
	usart_tx_at(USART_SERIAL_SIM900, AT_CIPSHUT);
	delay_s(1);
	usart_tx_at(USART_SERIAL_SIM900, AT_CIPSTATUS);
	delay_s(1);
	usart_tx_at(USART_SERIAL_SIM900, AT_CIPMUX);
	delay_s(1);
	
	int i = 0;
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
	char AT_MESSAGE2 = "\0x40\r";
	//usart_tx_at(USART_SERIAL_SIM900, AT_MESSAGE2);
	//void mqtt_connect();
	mqtt_packet(1, "10.8.10.136");
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
	*/
	
	/*
	//usart_tx_at(USART_SERIAL_EXAMPLE, CR);
	usart_tx_at(USART_SERIAL_SIM900, CR);
	delay_s(1);
	*/
	
	/*
	//usart_tx_at(USART_SERIAL_EXAMPLE, AT_CMGF);
	usart_tx_at(USART_SERIAL_SIM900, AT_CMGF);
	//STATUS = at_response(USART_SERIAL_SIM900);
	if (at_response(USART_SERIAL_SIM900))
	{
		led_blink(1);
		STATUS = 0;
	}
	*/
	/*
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
	
	
	
	
	
	/////////////////////
	
	
	gprs_states_t state = CIPSHUT;
	gprs_states_t next_state = state;
	
	while (true) {
		
		switch(state)
		{
			case CIPSHUT:
				next_state = CIPSTATUS;
				break;
			case CIPSTATUS:
				break;
		}
		state = next_state;
	}
}
