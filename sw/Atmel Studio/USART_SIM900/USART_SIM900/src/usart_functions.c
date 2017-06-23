/*
 * usart_functions.c
 *
 * Created: 15/05/2017 16:06:08
 *  Author: jan.rune.herheim
 */ 

#include <conf_usart_example.h>

//Functions declarations
void usart_tx_at(USART_t *usart, uint8_t *cmd);
uint8_t at_response(USART_t *usart);

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
