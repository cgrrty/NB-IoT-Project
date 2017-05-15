/*
 * sim900_at_commands.h
 *
 * Created: 11/05/2017 07:59:39
 *  Author: jan.rune.herheim
 */ 

#include <user_network_params.h>
#include <string.h>

#ifndef SIM900_AT_COMMANDS_H_
#define SIM900_AT_COMMANDS_H_

//Special AT characters
#define CR "\r" //carriage return
#define LF "\n" //line feed
#define CTRL_Z "\x1a" //ctrl+z
#define QUOTE "\"" //     "
#define DELIM "\",\"" //   ","


//General AT commands
#define AT "AT\r"
#define AT_CGATT "AT+CGATT=<1>"

//AT SMS commands
#define AT_CMGF "AT+CMGF=1\r"
#define AT_CMGS "AT+CMGS=\""SMS_RECEIVER"\"\r"
#define AT_MESSAGE "SIM900 Lillebakk\r"

//AT GPRS commands
#define AT_CIPSHUT "AT+CIPSHUT\r" //resets/close IP server connection
#define AT_CIPSTATUS "AT+CIPSTATUS\r" //check if IP stack is initialized
#define AT_CIPMUX "AT+CIPMUX=0\r" //0: single connection mode
#define AT_CSTT_INIT "AT+CSTT="
#define LEN_CSTT 9 //find another solution
char *AT_CSTT[] = {AT_CSTT_INIT,QUOTE,APN,DELIM,USERNAME,DELIM,PASSWORD,QUOTE,CR}; //connect to APN
#define AT_CIICR "AT+CIICR\r" //start wireless connection with GPRS
#define AT_CIFSR "AT+CIFSR\r" //return the local IP address (sim card)
#define AT_CIPSTART_INIT "AT+CIPSTART=" //didn't add strcat function, so the concenatiation is done during transmit.
#define LEN_CIPSTART 9
char *AT_CIPSTART[LEN_CIPSTART] = {AT_CIPSTART_INIT,QUOTE,IP_MODE,DELIM,SERVER_ADDR,DELIM,SERVER_PORT,QUOTE,CR};
#define AT_CIPSEND "AT+CIPSEND\r" //init send mode. THis command will returnm ">", and hence the data to be sent could be transmitted to the module.
								//It must be terminated with ctrl+z.
								//There will be a response from the server.



//define AT commands' response
#define RESPONSE_HEADER "\r\n"
#define RESPONSE_FOOTER "\r\n"
#define RESPONSE_OK "OK"


#endif /* SIM900_AT_COMMANDS_H_ */