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
#define CR_UINT8 0x0d
#define LF "\n" //line feed
#define CR_UINT8 0x0a
#define CTRL_Z "\x1a" //ctrl+z
#define QUOTE "\"" //     "
#define DELIM "\",\"" //   ","


//General AT commands
#define AT_AT "AT\r"
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
uint8_t LEN_CSTT = 9; //find another solution
char *AT_CSTT[] = {AT_CSTT_INIT,QUOTE,APN,DELIM,USERNAME,DELIM,PASSWORD,QUOTE,CR}; //connect to APN
//#define AT_CSTT "AT+CSTT=\"apn1.lillebakk.com\",\"\",\"\"\r"
#define AT_CIICR "AT+CIICR\r" //start wireless connection with GPRS
#define AT_CIFSR "AT+CIFSR\r" //return the local IP address (sim card)
#define AT_CIPSTART_INIT "AT+CIPSTART=" //didn't add strcat function, so the concenatiation is done during transmit.
uint8_t LEN_CIPSTART = 9;
char *AT_CIPSTART[] = {AT_CIPSTART_INIT,QUOTE,IP_MODE,DELIM,SERVER_ADDR,DELIM,SERVER_PORT,QUOTE,CR};
#define AT_CIPSEND "AT+CIPSEND\r" //init send mode. THis command will returnm ">", and hence the data to be sent could be transmitted to the module.
								//It must be terminated with ctrl+z.
								//There will be a response from the server.


//M95 response times
#define RESPONSE_TIME_300M 700 //300ms
#define RESPONSE_TIME_20S 40000 //20sec 

//AT TCP/IP commands M95
#define AT_QICLOSE "AT+QICLOSE\r"  //Same as AT_CIPSHUT, check notes in app note
#define AT_QISTAT "AT+QISTAT\r" //Same as AT_CIPSTATUS, or...A LOT OF STAT COMMANDS!!!! CHECK OUT!!!!!


//network status
#define AT_QNSTATUS "AT+QNSTATUS\r"

//ch 3.1 in TCP/IP app note
#define AT_QIFGCNT "AT+QIFGCNT=0\r" //config uart id
#define AT_QICSGP "AT+QICSGP=1,\"apn1.lillebakk.com\",\"\",\"\"\r"
#define AT_QIMUX "AT+QIMUX=0\r" //same as AT_CIPMUX
#define AT_QIMODE "AT+QIMODE=0\r"
#define AT_QIDNSIP "AT+QIDNSIP=0\r"

//ch 3.2 in TCP/IP app note
#define AT_QIREGAPP "AT+QIREGAPP\r"
#define AT_QIACT "AT+QIACT\r"
#define AT_QILOCIP "AT+QILOCIP\r"
#define AT_QIOPEN "AT+QIOPEN=\"TCP\",\"10.18.0.39\",1883\r"

//CH 3.4 in tcp/ip app note
#define AT_QISRVC "AT+QISRVC=1\r"
#define AT_QISEND "AT+QISEND\r"  //init send mode. THis command will returnm ">", and hence the data to be sent could be transmitted to the module.
//It must be terminated with ctrl+z.
//There will be a response from the server.

//ch 3.5 in tcp/ip appnote
#define AT_QIDEACT  "AT+QIDEACT\r"

//Other commands
//AT+CBC for supply voltage


/*
#define AT_QIREGAPP_INIT "AT+QICSGP="
uint8_t LEN_QIREGAPP = 11;
char *AT_QIREGAPP[] = {"1",DELIM,QUOTE,APN,DELIM,USERNAME,DELIM,PASSWORD,QUOTE,CR}; //connect to APN, same as AT_CSTT
	*/
//MISSING SOMETHING HERE??????
/*
#define AT_QIOPEN_INIT "AT+QIOPEN=" //didn't add strcat function, so the concenatiation is done during transmit.
uint8_t LEN_QIOPEN = 9;
char *AT_QIOPEN[] = {AT_QIOPEN_INIT,QUOTE,IP_MODE,DELIM,SERVER_ADDR,DELIM,SERVER_PORT,QUOTE,CR};
	*/




//define AT commands' response
#define RESPONSE_HEADER "\r\n"
#define RESPONSE_FOOTER "\r\n"
#define RESPONSE_OK "OK"
#define RESPONSE_ERROR "\r\nTIMEOUT ERROR\r\n"
#define O_UINT8 0x4f
#define K_UINT8 0x4b

//define other text commands
#define SPACE " "
#define ENTER_SLEEP "\n\nENTER SLEEP\n\n"


#endif /* SIM900_AT_COMMANDS_H_ */