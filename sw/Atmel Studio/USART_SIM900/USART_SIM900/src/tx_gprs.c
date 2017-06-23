/*
 * tx_gprs.c
 *
 * Created: 15/05/2017 15:53:19
 *  Author: jan.rune.herheim
 */ 

#include <stdio.h>
#include "tx_gprs.h"
#include <conf_usart_example.h>
#include <usart_functions.c>


int tx_gprs(char* message) {
	
	gprs_states_t gprs_state = CIPSHUT_INIT;
	gprs_states_t gprs_next_state = gprs_state;
	
	uint8_t tx_active = 1;
		
	while (tx_active) {
	
		switch(gprs_state) //compare against controller state????
		{
			case CIPSHUT_INIT: {
				gprs_next_state = CIPSTATUS;
				
				usart_tx_at(USART_SERIAL_SIM900, AT_CIPSHUT);
				delay_s(1);
				
				break;
			}
						
			case CIPSTATUS: {
				gprs_next_state = CIPMUX;
					
				usart_tx_at(USART_SERIAL_SIM900, AT_CIPSTATUS);
				delay_s(1);
				
				break;
			}
										
			case CIPMUX: {
				gprs_next_state = CSTT;
				
				usart_tx_at(USART_SERIAL_SIM900, AT_CIPMUX);
				delay_s(1);
				
				break;
			}
			
			case CSTT: {
				gprs_next_state = CIICR;
				
				int i = 0;
				while (i < LEN_CSTT)
				{
					usart_tx_at(USART_SERIAL_SIM900, AT_CSTT[i]);
					i++;
				}
				delay_s(1);
				
				break;
			}
						
			case CIICR: {
				gprs_next_state = CIFSR;
				
				usart_tx_at(USART_SERIAL_SIM900, AT_CIICR);
				delay_s(3);
				
				break;
			}
						
			case CIFSR: {
				gprs_next_state = CIPSTART;
				
				usart_tx_at(USART_SERIAL_SIM900, AT_CIFSR);
				delay_s(1);
				
				break;
			}
			
			case CIPSTART: {
				gprs_next_state = CIPSEND;
				
				i = 0;
				while (i < LEN_CIPSTART)
				{
					usart_tx_at(USART_SERIAL_SIM900, AT_CIPSTART[i]);
					i++;
				}
				delay_s(3);
				
				break;
			}
						
			case CIPSEND: {
				gprs_next_state = CIPSHUT;
				
				usart_tx_at(USART_SERIAL_SIM900, AT_CIPSEND);
				delay_s(1);
				char* AT_MESSAGE2 = "0 0 0 512 1023 125";
				//usart_tx_at(USART_SERIAL_SIM900, AT_MESSAGE2);
				//void mqtt_connect();
				mqtt_packet(AT_MESSAGE2);
				delay_s(1);
				usart_tx_at(USART_SERIAL_SIM900, CTRL_Z);
				delay_s(1);
				
				break;
			}
			
			case CIPSHUT: {
				gprs_next_state = CIPSHUT_INIT;
				tx_active = 0; //done for now...
							
				usart_tx_at(USART_SERIAL_SIM900, AT_CIPSHUT);
				
				break;
			}
						
			default:
			/*
			need to figure out which statements/stage to enter if this occurs.
			It will be dependent on the error message. 
			Go to sleep?
			Measure again?
			Transmit again?
			Other?
			*/
			gprs_next_state = CIPSHUT_INIT;
			tx_active = 0;
			break;
		}
		gprs_state = gprs_next_state;
	}
	
	led_blink();
	//done
	return 0;
}