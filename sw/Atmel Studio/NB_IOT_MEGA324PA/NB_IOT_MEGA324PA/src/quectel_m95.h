/*
 * sim900_at_commands.h
 *
 * Created: 11/05/2017 07:59:39
 *  Author: jan.rune.herheim
 */ 


#ifndef SIM900_AT_COMMANDS_H_
#define SIM900_AT_COMMANDS_H_


#include "user_network_params.h"
#include "stdint.h"
//#include "asf.h"

typedef struct m95_at {
	uint8_t *cmd;
	char *comp;
	//char *comp_er;
	uint32_t resp_time;
	uint8_t retries;
} m95_at_t;

//DEFINITION OF M95 NETWORK CONNECTION
#define M95_CONNECT_NR_COMMANDS 16 //number of AT commands to execute

//FUNCTION DEFINITIONS
// static const m95_at_t m95_connect[M95_CONNECT_NR_COMMANDS];
// static const m95_at_t m95_tx[M95_CONNECT_NR_COMMANDS];
// static const m95_at_t m95_disconnect[M95_CONNECT_NR_COMMANDS];
//////////////////////////////////////////////////////////////////////////



//Special AT characters
#define CR "\r" //carriage return
//#define CR_UINT8 0x0d
#define LF "\n" //line feed
#define CR_UINT8 0x0a
#define CTRL_Z "\x1a" //ctrl+z
//const char CTRL_Z = "\x1a"; //ctrl+z
#define QUOTE "\"" //     "
#define DELIM "\",\"" //   ","


//General AT commands
#define AT_AT "AT\r"
#define AT_CGATT "AT+CGATT=<1>"
#define  AT_QPWD_1 "AT+QPOWD=1\r" //power off normal mode
#define  AT_QPWD_0 "AT+QPOWD=0\r" //power off emergency

//AT SMS commands
#define AT_CMGF "AT+CMGF=1\r"
#define AT_CMGS "AT+CMGS=\""SMS_RECEIVER"\"\r"
#define AT_MESSAGE "SIM900 Lillebakk\r"

//AT GPRS commands
#define AT_CIPSHUT "AT+CIPSHUT\r" //resets/close IP server connection
#define AT_CIPSTATUS "AT+CIPSTATUS\r" //check if IP stack is initialized
#define AT_CIPMUX "AT+CIPMUX=0\r" //0: single connection mode
#define AT_CSTT_INIT "AT+CSTT="

//#define AT_CSTT "AT+CSTT=\"apn1.lillebakk.com\",\"\",\"\"\r"
#define AT_CIICR "AT+CIICR\r" //start wireless connection with GPRS
#define AT_CIFSR "AT+CIFSR\r" //return the local IP address (sim card)
#define AT_CIPSTART_INIT "AT+CIPSTART=" //didn't add strcat function, so the concenatiation is done during transmit.

#define AT_CIPSEND "AT+CIPSEND\r" //init send mode. THis command will returnm ">", and hence the data to be sent could be transmitted to the module.
								//It must be terminated with ctrl+z.
								//There will be a response from the server.

	//AT+CREG??????????
	
	//SETTING RESPONSE FORMAT
// 	#define ATV0 "ATV0\r" //response format is numbers
// 	#define ATV1 "ATV1\r" //response format is text


//the number of times an AT command will be sent and the delay between repeats.

#define AT_REPEAT 1
#define AT_REPEAT_LONG 27

#define DELAY_1MS_BASE 12 //333 * 1us to get 1ms... FIX!!!!!!!!!!!!!!!!!!!!!!!!!!!
//#define AT_REPEAT_DELAY 200 //1s ~ delay(333) => 1ms ~ delay_ms(0.33). WHY I DON'T KNOW!!!!!

//M95 response times
#define RESPONSE_TIME_30M 3 // (uint32_t)DELAY_1MS_BASE*30 //1s ~ delay(333) => 1ms ~ delay_ms(0.33). WHY I DON'T KNOW!!!!!
#define RESPONSE_TIME_300M 30 //(uint32_t)DELAY_1MS_BASE*300 //1s ~ delay(333) => 1ms ~ delay_ms(0.33). WHY I DON'T KNOW!!!!!
#define RESPONSE_TIME_20S 200 // (uint32_t)DELAY_1MS_BASE*20000 //20*1000 //20sec 

//AT TCP/IP commands M95
#define AT_QICLOSE "AT+QICLOSE\r"  //Same as AT_CIPSHUT, check notes in app note
#define AT_QISTAT "AT+QISTAT\r"
#define AT_QISTAT_COMPARE_IP_START "IP START\r\n"
#define AT_QISTAT_COMPARE_CONNECT_OK "CONNECT OK\r\n"
#define AT_QISTAT_COMPARE_IP_INITIAL "IP INITIAL\r\n"
#define AT_QISTAT_COMPARE_IP_STATUS "IP STATUS\r\n"
#define AT_QISTAT_COMPARE_IP_CLOSE "IP CLOSE\r\n"
#define AT_QISTAT_COMPARE_IP_GPRSACT "IP GPRSACT\r\n"
#define AT_QISTAT_COMPARE_TCP_CONNECTING "TCP CONNECTING\r\n"
#define AT_QISTAT_COMPARE_UDP_CONNECTING "UDP CONNECTING\r\n"
#define AT_QPOWD "AT+QPOED\r"


//network and radio status
#define AT_QNSTATUS "AT+QNSTATUS\r"
#define AT_QNSTATUS_COMPARE "QNSTATUS: 0\r\n"
#define AT_QNSTATUS_COMPARE_255 "QNSTATUS: 255\r\n"
#define AT_QNSTATUS_COMPARE_1 "QNSTATUS: 1\r\n"
#define AT_QNSTATUS_COMPARE_2 "QNSTATUS: 2\r\n"

#define AT_QLTS "AT+QLTS\r" //Time and date from network.
#define AT_QLTS_COMPARE  "\"\r\n" //"QLTS: "
#define AT_QLTS_START 18 //if response is ok the time starts at position 18.
#define AT_CBC "AT+CBC\r" //battery voltage

//ch 3.1 in TCP/IP app note
#define AT_QIFGCNT "AT+QIFGCNT=0\r" //config uart id
#define AT_QICSGP "AT+QICSGP=1,\"apn1.lillebakk.com\",\"\",\"\"\r"
#define AT_QIMUX "AT+QIMUX=0\r" //same as AT_CIPMUX
#define AT_QIMODE "AT+QIMODE=0\r"
#define AT_QIDNSIP "AT+QIDNSIP=0\r"

//ch 3.2 in TCP/IP app note
#define AT_QIREGAPP "AT+QIREGAPP\r"
#define AT_QIACT "AT+QIACT\r"
#define AT_QIACT_COMPARE "QIACT"
#define AT_QILOCIP "AT+QILOCIP\r"
#define AT_QIOPEN "AT+QIOPEN=\"TCP\",\"10.18.0.39\",1883\r"

//CH 3.4 in tcp/ip app note
#define AT_QISRVC "AT+QISRVC=1\r"
#define AT_QISEND "AT+QISEND\r"  //init send mode. THis command will returnm ">", and hence the data to be sent could be transmitted to the module.
#define AT_QISEND_COMPARE "> "
#define AT_CTRLZ_COMPARE "SEND OK\r"
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
#define RESPONSE_OK "OK\r\n"
#define RESPONSE_ERROR "\r\nTIMEOUT ERROR\r\n"
#define O_UINT8 0x4f
#define K_UINT8 0x4b
#define QNSTATUS_0 "QNSTATUS: 0"

//define other text commands
#define SPACE " "
#define ENTER_SLEEP "\r\nENTER SLEEP\r\n"
#define TX_STATEMENT "\r\nTx returned status code: "

const uint8_t LEN_CSTT PROGMEM = 9; //find another solution
const char *AT_CSTT[] = {AT_CSTT_INIT,QUOTE,APN,DELIM,USERNAME,DELIM,PASSWORD,QUOTE,CR}; //connect to APN
const uint8_t LEN_CIPSTART = 9;
const char *AT_CIPSTART[] = {AT_CIPSTART_INIT,QUOTE,IP_MODE,DELIM,SERVER_ADDR,DELIM,SERVER_PORT,QUOTE,CR};


const static m95_at_t m95_connect[] = {
	//SEQUENCE MUST MATCH TO VISIO, AND WHAT IS GIVEN FOR THE M95
	{AT_QNSTATUS, AT_QNSTATUS_COMPARE, RESPONSE_TIME_300M, AT_REPEAT_LONG}, //repeat long due to long time to connect to network
	{AT_QIFGCNT, RESPONSE_OK, RESPONSE_TIME_300M, AT_REPEAT},
	{AT_QICSGP, RESPONSE_OK, RESPONSE_TIME_300M, AT_REPEAT},
	{AT_QIMUX, RESPONSE_OK, RESPONSE_TIME_300M, AT_REPEAT},
	{AT_QIMODE, RESPONSE_OK, RESPONSE_TIME_300M, AT_REPEAT},
	{AT_QIDNSIP, RESPONSE_OK, RESPONSE_TIME_300M, AT_REPEAT},
	{AT_QIREGAPP, RESPONSE_OK, RESPONSE_TIME_300M, AT_REPEAT},
		
	{AT_QISTAT, AT_QISTAT_COMPARE_IP_START, RESPONSE_TIME_300M, AT_REPEAT}, //NEED LONG??????
	{AT_QIACT, RESPONSE_OK, RESPONSE_TIME_20S, AT_REPEAT_LONG}, //NEED STATUS IP START
		
	/*
	{AT_QISTAT, AT_QISTAT_COMPARE_IP_STATUS ||
				AT_QISTAT_COMPARE_IP_GPRSACT ||
				AT_QISTAT_COMPARE_TCP_CONNECTING ||
				AT_QISTAT_COMPARE_UDP_CONNECTING ||
				AT_QISTAT_COMPARE_CONNECT_OK ||
				AT_QISTAT_COMPARE_IP_CLOSE, RESPONSE_TIME_300M, AT_REPEAT}, //NEED LONG???? DOESNT WORK WITH ALL THE ORING!!!!!!!!! FIX!!!!!!!!
				*/
	{AT_QISTAT, AT_QISTAT_COMPARE_IP_GPRSACT, RESPONSE_TIME_300M, AT_REPEAT}, //NEED LONG??????
	{AT_QILOCIP, LOCAL_IP, RESPONSE_TIME_300M, AT_REPEAT_LONG}, //NEED IP GPRSACT, IPSTATUS, TCP/UDP CONNECTING, CONNECT OK IP CLOSE
	
	/*
	{AT_QISTAT, AT_QISTAT_COMPARE_CONNECT_OK || 
				AT_QISTAT_COMPARE_IP_INITIAL ||
				AT_QISTAT_COMPARE_IP_STATUS ||
				AT_QISTAT_COMPARE_IP_CLOSE, RESPONSE_TIME_300M, AT_REPEAT_LONG}, //NEED LONG???? DOESNT WORK WITH ALL THE ORING!!!!!!!!! FIX!!!!!!!!
				*/
	{AT_QISTAT, AT_QISTAT_COMPARE_IP_STATUS, RESPONSE_TIME_300M, AT_REPEAT}, //NEED LONG??????
	{AT_QIOPEN, RESPONSE_OK, RESPONSE_TIME_20S, AT_REPEAT},
		
	{AT_QISRVC, RESPONSE_OK, RESPONSE_TIME_300M, AT_REPEAT},
	{AT_QLTS, AT_QLTS_COMPARE, RESPONSE_TIME_300M, AT_REPEAT}

};

static const m95_at_t m95_tx[3] = {
	{AT_QISTAT, AT_QISTAT_COMPARE_CONNECT_OK, RESPONSE_TIME_300M, AT_REPEAT_LONG}, //NEED LONG??????
	{AT_QISEND, AT_QISEND_COMPARE, RESPONSE_TIME_300M, AT_REPEAT},
	{CTRL_Z, AT_CTRLZ_COMPARE, RESPONSE_TIME_300M, AT_REPEAT}
};

static const m95_at_t m95_disconnect[] = {
	{AT_QICLOSE, RESPONSE_OK, RESPONSE_TIME_300M, AT_REPEAT},
	{AT_QIDEACT, RESPONSE_OK, RESPONSE_TIME_20S, AT_REPEAT}
};



#endif /* SIM900_AT_COMMANDS_H_ */