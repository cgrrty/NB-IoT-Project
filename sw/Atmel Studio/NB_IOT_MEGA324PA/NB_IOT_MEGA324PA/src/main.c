/*
 * NB_IOT_MEGA324PA.c
 *
 * Created: 09/07/2017 19:57:45
 * Author : jan.rune.herheim
 */ 

#include <avr/io.h>
/*
 
 */
//#include "main.h"
/*
 * main.h
 *
 * Created: 28/06/2017 17:09:19
 *  Author: jan.rune.herheim
 */ 

#include "main.h"

/*
	TODO:
	- HW data flow
	*/

//Define the CPU clock speed.
//#define F_CPU 1000000UL // Clock Speed

/*
Data shared between the ISR and your main program must be both volatile and global in scope in the C language. 
Without the volatile keyword, the compiler may optimize out accesses to a variable you update in an ISR,
as the C language itself has no concept of different execution threads. Take the following example:
*/

//MOVE!!!!
#define USART1    (*(USART_t *)0xC8) 
#define USART_RADIO				 &USART0 //
#define USART_TERMINAL			&USART1 //
#define  USART_EXT_DATA			&USART1


#define BAUD 4800 //19200
#define MYUBRR F_CPU/16/BAUD-1

//PUT IN OWN FILE???
#define USART_BAUDRATE    BAUD //19200
#define USART_CHAR_LENGTH         USART_CHSIZE_8BIT_gc
#define USART_PARITY              USART_PMODE_DISABLED_gc
#define USART_STOP_BIT            true

// USART for debug (COM port)
static usart_rs232_options_t USART_OPTIONS = {
	.baudrate = USART_BAUDRATE,
	.charlength = USART_CHAR_LENGTH,
	.paritytype = USART_PARITY,
	.stopbits = false //USART_STOP_BIT
};

//DFEINITIOINS OF CONTROLLER STATES. THESE WILL MATCH WITH VISIO FLOW CHART.
typedef enum controller_states {
	READ_EXT_DATA,
	MEASURE,
	CALC,
	STORE_EXT_MEM,
	RF_POWER_ON,
	RF_CONNECT,
	GENERATE_PACKAGE,
	TX_DATA,
	RX_DATA,
	RF_DISCONNECT,
	RF_POWER_OFF,
	RESET_REGISTERS
} controller_states_t;

//Initial states.
controller_states_t controller_state = READ_EXT_DATA; //CHECK IN FINAL.
controller_states_t controller_next_state = READ_EXT_DATA; //
//////////////////////////////////////////////////////////////////////////


/*DECLARATION OF WAKEUP AND TRANSFER RATES IN SECONDS
Default settings are 5s wakeup rate and 600s (10 minutes) transfer rate.
*/
#define WAKEUP_RATE SAMPLING_TIME //5 //Use the value defined for the load cell in this case.
#define TRANSMIT_RATE AVERAGING_TIME //600 //use the value defined for the load cell in this case.
//NEED A DEDICATED COUNTER ON MEGA TO MAKE RTC TRHOUGH THE WDT
uint16_t wdt_counter = 1; //minimum at 1 second
uint16_t transmit_counter = 0;
//////////////////////////////////////////////////////////////////////////




//DEFINE DATA POSITIONS
#define POSITION_ANA0 POSITION_CURRENT //0
#define POSITION_ANA1 POSITION_PREV //1
#define POSITION_ANA2 POSITION_AVG //2
#define POSITION_ANA3 POSITION_MIN //3
#define POSITION_ANA4 POSITION_MAX //4
#define POSITION_ANA5 POSITION_TRAN_MAX //5
#define POSITION_TEMP 6
#define POSITION_VDD 7
#define POSITION_DIO 8
#define POSITION_TIME POSITION_ACCU_CNT //9
#define POSITION_YEAR 10
#define POSITION_MONTH 11
#define POSITION_DAY 12
#define POSITION_HOUR 13
#define POSITION_MINUTE 14
#define POSITION_SECOND 15
#define POSITION_STATUS 16
#define TX_DATA_SIZE 17 //sum of the above

//DEFINE DATA RESET VALUES
#define RESET_VALUE_ANA0 0
#define RESET_VALUE_ANA1 0
#define RESET_VALUE_ANA2 0
#define RESET_VALUE_ANA3 RESET_VALUE_MIN //0
#define RESET_VALUE_ANA4 0
#define RESET_VALUE_ANA5 0
#define RESET_VALUE_TEMP 0
#define RESET_VALUE_VDD 0
#define RESET_VALUE_DIO 0x0000 //all bits cleared, i.e. all ok.
#define RESET_VALUE_TIME 0
#define RESET_VALUE_YEAR 0
#define RESET_VALUE_MONTH 0
#define RESET_VALUE_DAY 0
#define RESET_VALUE_HOUR 0
#define RESET_VALUE_MINUTE 0
#define RESET_VALUE_SECOND 0
#define RESET_VALUE_STATUS 0x00 //all bits cleared, i.e. all ok.

//DEFINE STATUS BITS
#define STATUS_BIT_RF_POWER_ON 0
#define STATUS_BIT_RF_CONNECT 1
#define STATUS_BIT_TX 2

volatile uint16_t tx_data[TX_DATA_SIZE]; //declare the array for internal accumulation and storage.
const static uint16_t tx_data_reset_values[TX_DATA_SIZE] = {RESET_VALUE_ANA0, RESET_VALUE_ANA1, RESET_VALUE_ANA2, RESET_VALUE_ANA3,
	RESET_VALUE_ANA4, RESET_VALUE_ANA5, RESET_VALUE_TEMP, RESET_VALUE_VDD, RESET_VALUE_DIO, RESET_VALUE_TIME,
	RESET_VALUE_YEAR, RESET_VALUE_MONTH, RESET_VALUE_DAY, RESET_VALUE_HOUR, RESET_VALUE_MINUTE, RESET_VALUE_SECOND, RESET_VALUE_STATUS}; //declare the reset register.
//////////////////////////////////////////////////////////////////////////

//RADIO RESPONSE ARRAY AND SIZE DEFINITIONS
#define RESPONSE_SIZE 128
volatile char response[RESPONSE_SIZE];
volatile uint8_t response_counter = 0;
volatile uint32_t response_timeout = 0;
volatile uint32_t response_timeout_counter = 0;

volatile uint8_t *tx_char = 0;
//////////////////////////////////////////////////////////////////////////

//DEFINITIONS OF THE EXTERNAL DATA PINS
#define REQUEST_DATA_PORT PORTD
#define REQUEST_DATA_PIN 7
//////////////////////////////////////////////////////////////////////////

//SPECIALIZED PARAMETERS FOR THE LOAD CELL APPLICATION
//local data declarations
uint32_t accu_data = 0; //allocating internal accumulation storage.
//////////////////////////////////////////////////////////////////////////

//DEFINITIONS OF STATUS AND MODES PARAMETERS
uint8_t RTC_ISR_ACTIVE = 0;
//////////////////////////////////////////////////////////////////////////

//DEFINITIONS OF PINS CONNECTED TO THE RADIO
#define PWRKEY_PORT PORTD //PORTE
#define PWRKEY_PIN 4 //6
#define STATUS_PORT PIND //PORTE
#define STATUS_PIN 5 //7
//#define NETLIGHT_PORT PORTR
//#define NETLIGHT_PIN 0
//////////////////////////////////////////////////////////////////////////

//DEFINITIONS OF PINS CONNECTED TO THE LOADCELL
// #define LOADCELL_PWR_PORT PORTD
// #define LOADCELL_PWR_PIN 6
#define LOADCELL_PWR_PORT PORTA
#define LOADCELL_PWR_PIN0 1
#define LOADCELL_PWR_PIN1 2
//////////////////////////////////////////////////////////////////////////

//ADC definitions
#define ADC_CLOCK_SPEED 200000UL //ADC clock speed
#define ADC_NUM_AVG 9 //number of averages
#define ADC_OFFSET 0 //CALIBRATION DATA. Measured to be 0 @ 0V on the current device. Should probably be measured at 0 load condition.
#define ADC_GAIN_ERROR 0.98 //CALIBRATION DATA. Measured at loadcell 0 load condition (0,990V). ADC = vin/vref * 2^n, which is 1584 @ vref=2,56 and 12 bits result. Should probably be measured at full load condition.
//////////////////////////////////////////////////////////////////////////

//DEFINITIONS OF DATA FORMATS AND DATA SIZES TO BE TRANSMITTED THROUGH THE RADIO
#define TRANSFER_DATA_BASE 10 //DATA FORMAT THE DATA IS TRANSFERRED WITH. 10 = Base10 (decimal), 16 = Base16 (hex), 32 = Base32
#define TX_DATA_DIGITS 4 //NUMBER OF DATA DIGITS TO TRANSMIT: 2 or 4
#define TX_DATE_DIGITS 2 //Number of date digits to transmit: 1 or 2
#define TX_ASCII 1 //0 will transfer hex bytes, 1 will transfer ascii coded bytes: 1 or 2

#define TRANSFER_DATA_SIZE 100 //ASSUMING 128 bytes are enough.....
//char tx_data_bytes[TRANSFER_DATA_SIZE] = ""; //Final data to be transmitted (and stored?)
//char tx_data_bytes[TX_DATA_SIZE][5]; //TX_DATA_SIZE amount of strings with length of 5 each.
char tx_data_bytes[5]; //TX_DATA_SIZE amount of strings with length of 5 each.
//char tx_data_topics[6] = "ABCDE"; //TX_DATA_SIZE amount of strings with length of 5 each.
#define TRANSFER_DATA_SIZE_PACKAGE (TRANSFER_DATA_SIZE) //ASSUMING 2 TIMES THE DATA SIZE IS THE TOTAL PACKAGE OVERHEAD.
char tx_data_package[TRANSFER_DATA_SIZE_PACKAGE] = ""; //Final data package generated from the tx_data_bytes array.
//char test_data_package[33] = {0x10, 0x12, 0x00, 0x4, 0x4d, 0x51, 0x54, 0x54, 0x4, 0xc2, 0x00, 0x14, 0x00, 0x2, 0x4c, 0x45, 0x00, 0x00, 0x00, 0x00, 0x30, 0x09, 0x00, 0x4, 0x4c, 0x45, 0x2f, 0x30, 0x36, 0x37, 0x39, 0xe0, 0x00};
int transfer_data_length_package = 0; //ACTUAL PACKAGE SIZE TO BE TRANSMITTED. CALCULATED IN PROGRAM. KEEP AS LOW AS POSSIBLE!!!!!!
int transfer_data_package_counter = 0; //amount of packages transferred.
////////////////////////////////////////////////////////////////////////////////////


//Functions related to the radio communication
void usart_tx_at(USART_t *usart, uint8_t *cmd) {
	
	//send the command
	while(*cmd) {
		usart_putchar(usart, *cmd++);
	}
	
}

uint8_t reset_tx_data(uint16_t *array, uint16_t *reset_array, uint8_t len_array) {
	uint8_t status = 0;
	uint8_t i = 0;
	
	while (i < len_array)
	{
		*(array+i) = *(reset_array+i);
		i++;
	}
	return status;
}

void reset_char_array(char *array_pointer , uint8_t size) {
	uint8_t i = 0;
	while (i < size)
	{
		*(array_pointer+i) = 0x00;
		i++;
	}
}


uint8_t reset_all_data() {
	//reset data and date
	reset_char_array(&tx_data_bytes, TRANSFER_DATA_SIZE);
	reset_char_array(&tx_data_package, TRANSFER_DATA_SIZE_PACKAGE);
	reset_tx_data(&tx_data, &tx_data_reset_values, TX_DATA_SIZE); //why this one must be last to get correct reset values??????
}

void rtc_init_period(uint16_t period)
{
	//USE WDT ON MEGA
	
	MCUSR = 0x00; //RESET STATUS REGISTER
	WDTCSR = (1<<WDCE) | (1<<WDE);
	//WDTCSR = (1<<WDIE) | (1<<WDP2) | (1<<WDP1); //1 second wakeup
	WDTCSR = (1<<WDIE) | (1<<WDP3); //4 second wakeup
	//WDTCSR = (1<<WDIE) | (1<<WDP3) | (1<<WDP0); //8 second wakeup
	
	
}

void loadcell_pins_init(void) {
	//DDRD |= (1<<LOADCELL_PWR_PIN); //define power pin for external sensor. This must be a real switch in final application!!!!
	DDRA |= (1<<LOADCELL_PWR_PIN0); //define power pin for external sensor. This must be a real switch in final application!!!!
	//DDRA |= (1<<LOADCELL_PWR_PIN1); //define power pin for external sensor. This must be a real switch in final application!!!!
	LOADCELL_PWR_PORT &= ~(1<<LOADCELL_PWR_PIN0); //set to ground, i.e. power off.
	//LOADCELL_PWR_PORT &= ~(1<<LOADCELL_PWR_PIN1); //set to ground, i.e. power off.
}

void radio_pins_init(void) {
	
	//PWRKEY and startup sequence.
	DDRD |= (1<<PWRKEY_PIN); //reset pin
	//PWRKEY_PORT |= (1<<PWRKEY_PIN); //normal level for this pin
	
	
	//STATUS, NOT NEEDED AS NETLIGHT IS REQUIRED BEFORE SENDING COMMANDS.
	DDRD &= ~(1<<STATUS_PIN); //input
	//portctrl_setup(STATUS_PORT, STATUS_PIN); //should not be needed.
	
	//NETLIGHT. NOT AVAILABLE ON DEVELOPMENT BOARD, HAVE TO USE SW CALL TO CHECK FOR CONNECTION STATUS.
	//NETLIGHT_PORT.DIR &= ~(1<<NETLIGHT_PIN); //input
	
}

void my_delay_10ms(uint8_t loops)
{
  /* Prevents the use of floating point libraries. Delaying in groups of
     10ms increases accuracy by reducing the time overhead for each loop
     interation of the while.                                            */

	while (loops--)
	  delay_ms(10);
}

uint8_t loadcell_power_on(void) {
	LOADCELL_PWR_PORT |= (1<<LOADCELL_PWR_PIN0);
	//LOADCELL_PWR_PORT |= (1<<LOADCELL_PWR_PIN1);
	delay_ms(8); //Cap load = 1pF/cm assuming 20cm of wiring, and decoupling of 100uF. 10 in resistance. 12 bits voltage to settle requires 8 time constants. I.e. 8ms.
	//resistors and caps in the amps etc is assumed to have resistances of <1M and caps in the nF range
	//Startup times for the modules should be <1ms, but not verified.
}

uint8_t loadcell_power_off() {
	LOADCELL_PWR_PORT &= ~(1<<LOADCELL_PWR_PIN0);
	//LOADCELL_PWR_PORT &= ~(1<<LOADCELL_PWR_PIN1);
}

uint8_t radio_power_on(void) {
	uint8_t status = 0;
	uint8_t cnt_pwron = 0;
	
	PWRKEY_PORT &= ~(1<<PWRKEY_PIN); //normal state. PWRKEY pin is inverted compared to the one connected directly to the M95 due to the open collector driver of PWRKEY.
	delay_ms(100); //wait for battery voltage to settle.
	PWRKEY_PORT |= (1<<PWRKEY_PIN); //turn on
	delay_s(1); //boot time, 800ms recommended for m95
	PWRKEY_PORT &= ~(1<<PWRKEY_PIN); //normal state
	delay_s(1);
	//PWRKEY_PORT |= (1<<PWRKEY_PIN); //normal level for this pin
	
	//wait for status
	while ( ((STATUS_PORT & (1<<STATUS_PIN)) == 0) & (cnt_pwron < AT_REPEAT_LONG) )
	{
		delay_ms(200);
		cnt_pwron++;
	} 
	
	if (cnt_pwron == AT_REPEAT_LONG)
	{
		status = 1;
	}
	
	return status;
}

uint8_t radio_power_off(void) {
	uint8_t status = 0;
	uint8_t cnt_pwrdwn = 0;
	
	START:
	PWRKEY_PORT &= ~(1<<PWRKEY_PIN); //normal state, inverted logic due to open collector driver of PWRKEY on M95
	PWRKEY_PORT |= (1<<PWRKEY_PIN); //pull down on M95
	my_delay_10ms(80); //between 0,7s and 1s. Too narrow for delay routine????
	PWRKEY_PORT &= ~(1<<PWRKEY_PIN); //normal state, wait for status down.
	
	while (((STATUS_PORT & (1<<STATUS_PIN)) == 0x20)  & (cnt_pwrdwn < 12) )
	{
	 	delay_s(1);
		cnt_pwrdwn++;
	}
		
	if (cnt_pwrdwn == AT_REPEAT_LONG)
	{
		goto START;
		status = 1;
	}
		
	return status;
}


uint8_t radio_power_off_at(void) {
	uint8_t status = 0;
	uint8_t cnt_pwrdwn = 0;
	
	while (((STATUS_PORT & (1<<STATUS_PIN)) == 0x20)  & (cnt_pwrdwn < AT_REPEAT_LONG) )
	{
		#ifdef DEBUG
			usart_tx_at(USART_TERMINAL, AT_QPWD_1); //DEBUG
		#endif // DEBUG
		
		usart_tx_at(USART_RADIO, AT_QPWD_1); //normal power off
		
		delay_s(4);
		if (((STATUS_PORT & (1<<STATUS_PIN)) == 0x20)) //if still on
		{
			#ifdef DEBUG
				usart_tx_at(USART_TERMINAL, AT_QPWD_0); //DEBUG
			#endif // DEBUG
			
			usart_tx_at(USART_RADIO, AT_QPWD_0);
			delay_s(6);
		}
		cnt_pwrdwn++;
	}
	
	if (cnt_pwrdwn == AT_REPEAT_LONG)
	{
		status = 1;
	}
	
	return status;
}


#ifdef DEBUG
void led_blink(uint16_t on_time) {
	
	PORTB |= (1<<5);
	delay_ms(on_time);
	PORTB &= ~(1<<5);
	delay_ms(on_time);
}
#endif // DEBUG

static void adc_initialization(void)
{
	/*
	CHECK F_CPU AND ADC CLOCK SPEED SETTINGS!!!!!!
	The initial target is 1MHz CPU clock and <=200kHz ADC clock speed => div by >=5 => div by 8 => 125kHz ADC clock.
	*/
	//PRR0 &= ~(1<<PRADC); //enable ADC clock
	adc_init(ADC_PRESCALER_DIV8);
}

uint16_t adc_10_to_12_bits (uint8_t adc_ch) {
	
	/*
	From the specification the final resolution given today's system is 12,3 bits => assuming 12 bits.
	The oversampling and decimation rates would then be 16 times oversampling/accumulation and 2 times right shift (divide by 4).
	*/
	uint8_t i = 0;
	uint8_t num_oversample = 16;
	volatile uint16_t res = 0;
	//uint16_t res_accu[num_oversample];
	uint8_t j = 0;
	uint16_t res_avg = 0;
	
	
	signed int scale_factor = 16384; //2^14
	signed int factor = scale_factor*ADC_GAIN_ERROR;
	signed long correction = scale_factor*(0.5-(ADC_OFFSET*ADC_GAIN_ERROR));
			
	adc_initialization();
		
	while (i<num_oversample) //Sample and accumulate
	{
		res_avg = 0;
		j=0;
		while (j < 4) //average by 4
		{
			res_avg = res_avg + adc_read_10bit(adc_ch, ADC_VREF_2V56); //Measure with 2,56V as reference
			j++;
		}
		
		res_avg = (res_avg >> 2); //Divide by 4.
		res = res + res_avg; //accumulate
		
		#ifdef DEBUG
// 			reset_char_array(mynum, 5);
// 			itoa(res_accu[i], mynum, 10);
// 			usart_tx_at(USART_TERMINAL, mynum);
// 			usart_tx_at(USART_TERMINAL, RESPONSE_FOOTER);
		#endif // DEBUG
		i++;
	}
	
	res = (res >> 2); //decimate, right shift to get 12 bits results.
	res = (((((signed long)res*factor)+correction)<<2)>>16); //apply offset and gain error
		
	#ifdef DEBUG
		uint8_t mynum[5] = "";
		reset_char_array(mynum, 5);
		itoa(res, mynum, 10);
		usart_tx_at(USART_TERMINAL, RESPONSE_FOOTER);
		usart_tx_at(USART_TERMINAL, mynum);
		usart_tx_at(USART_TERMINAL, RESPONSE_FOOTER);
	#endif // DEBUG
	
	return res;
}


uint16_t controller_calc_avg(uint32_t data, uint16_t cnt) {
	uint16_t avg = 0;
	
	avg = data/cnt;
	return avg;
}

void at_get_radio_network_time(){
	
	int j = 0;
	int k = AT_QLTS_START;
	char temp[3] = "";
	
	while (j < 6)
	{
		int i = 0;
		while (i < 2)
		{
			temp[i] = *(response+k+i);
			i++;
		}
		tx_data[POSITION_YEAR+j] = atoi(temp);
		k = k+3;
		j++;
	}
	
}


uint8_t tx_at_response(const m95_at_t *opt) {
	
	uint8_t status = 0; //tx status, 0 = alles ok.
	uint8_t tx_at_cnt = 0; //nr of AT command sent
	char *ret; //response pointer
	//uint32_t i = 0;
			
	ret = 0;
	while (tx_at_cnt < opt->retries) //Less than nr of retries to send the AT command
	{
		reset_char_array(&response, RESPONSE_SIZE); //reset response buffer
		response_counter = 0; //RESET COUNTER
		response_timeout = opt->resp_time;
		response_timeout_counter = 0;
		
		usart_tx_at(USART_RADIO, opt->cmd); //send AT command to radio
		
		//READY FOR RECEIVING BYTES, ENABLE INTERRUPTS
		sei();
		usart_rx_complete_interrupt_enable(USART_RADIO);
		
		while (response_timeout_counter < response_timeout) {
			my_delay_10ms(1);
			ret = strstr(response, opt->comp); //DO THE COMPARISON AND BREAK THE LOOP
			if (ret != 0) //correct response received. IDEALLY IT SHOULD CHECK FOR WRONG RESPONSES TO AVOID TIMOUT TO BE RUN IF IT HAPPENS
			{
				status = 0;
				goto END;
				} else {
				status = 1;
			}
			
			response_timeout_counter++;
		}
		
		my_delay_10ms(30);
		tx_at_cnt++;
	}
	
	END:
	
	my_delay_10ms(1); //dummy delay for receiving even more data after compare match.
	usart_rx_complete_interrupt_disable(USART_RADIO);
	
	#ifdef DEBUG
	usart_tx_at(USART_TERMINAL, response); //DEBUG
	//usart_tx_at(USART_TERMINAL, RESPONSE_FOOTER); //DEBUG
	#endif // DEBUG
	
	return status;
}


uint8_t tx_data_response(char *data, int len) {
	
	uint8_t status = 0; //tx status, 0 = alles ok.
	//uint8_t tx_at_cnt = 0; //nr of AT command sent
	//char *ret; //response pointer
	uint8_t i = 0;
			
	reset_char_array(&response, RESPONSE_SIZE); //reset response buffer
	response_counter = 0; //RESET COUNTER

	while (i < len) //send AT command to radio
	{
		usart_putchar(USART_RADIO, *(data+i));
		//usart_putchar(USART_TERMINAL, *(data+i)); //DEBUG
		i++;
	}
	usart_putchar(USART_RADIO, 0x00);
	//usart_set_rx_interrupt_level(USART_RADIO, USART_INT_LVL_MED); //READY FOR RECEIVING BYTES
	sei();
	//UCSR0B |= (1 << 7); // Enable the USART Recieve Complete interrupt (USART_RXC)
	
	//wait for response
	usart_rx_complete_interrupt_enable(USART_RADIO);
	my_delay_10ms(30); //fixed delay. Could be made as a part of the AT command arrays...
	usart_rx_complete_interrupt_disable(USART_RADIO);
	
	#ifdef DEBUG
	usart_tx_at(USART_TERMINAL, response); //DEBUG
	//usart_tx_at(USART_TERMINAL, RESPONSE_FOOTER); //DEBUG
	#endif // DEBUG
	
	return status;
}


uint8_t data_to_char(uint16_t *array_data, uint8_t array_data_len, char *array_ascii, int base) {
	uint8_t status = 0;
	uint8_t i = 0; //REMOVE VOLATILE
	//uint8_t j = 0; //REMOVE VOLATILE
	char temp[5] = ""; //MAX 4 VALUES + NULL TERMINATION
	
	//CONVERT ALL 2 BYTES NUMBERS
	
	while (i <= (array_data_len-1))
	//while (i <= (POSITION_TIME & (array_data_len-1))) //WHY DON*T THIS ONE WORK? FIX!!!!
	{
		if (TX_ASCII)
		{
			itoa(*(array_data+i), temp, base); //CONVERT NUMBER TO ASCII
		} else {
			temp[0] = (*(array_data+i) >> 8) & 0xff; //JUST GRAB THE BYTES
			temp[1] = *(array_data+i) & 0xff;
		}
		
		strcpy((array_ascii+(i*5)), temp); //APPEND NUMBER. HOW TO AVOID THE 5????? FIX!!!!!!!!!!!!!!!!!!!!!!
		reset_char_array(&temp, sizeof(temp));
		i++;
	}
	//////////////////////////////////////////////////////////////////////////
	
	//CONVERT ALL 1 BYTES NUMBERS
	i = POSITION_YEAR;
	//while (i <= POSITION_STATUS)
// 	while (i <= (POSITION_STATUS & (array_data_len-1)))
// 	{
// 		if (TX_ASCII)
// 		{
// 			itoa(*(array_data+i), temp, base); //CONVERT NUMBER TO ASCII
// 			} else {
// 			temp[0] = *(array_data+i) & 0xff; //JUST GRAB THE BYTES
// 		}
// 		
// 		strcpy((array_ascii+(i*5)), temp); //APPEND NUMBER. HOW TO AVOID THE 5????? FIX!!!!!!!!!!!!!!!!!!!!!!
// 		reset_char_array(&temp, sizeof(temp));
// 		i++;
// 	}
// 	
	return status;
}

uint8_t at_rf_status(void) {
	/*
	status = 0 => all AT commands was executed sucesessfully
	status = 1 => one of the AT commands was not executed sucessfully.
	status = 32 => QLTS, i.e. the network time didn't execute sucessfully.
	
	WILL MATCH FLOWCHART IN VISIO
	*/
	
	uint8_t status = 0;
	uint8_t i = 0;
	//while (i < ((sizeof(m95_connect)/(sizeof(m95_connect[0])))-0))
	while (i < 1)
	{
		if (tx_at_response(&m95_status[i]) == 1) {status = 0; goto END;}
		i++;
	}
	
	#ifdef DEBUG
		//if (tx_at_response(&m95_status[1]) == 1) {status = 0; goto END;}
	#endif // DEBUG
		
	END: 
	
	return status;
}

uint8_t at_rf_gprs(void) {
	/*
	status = 0 => all AT commands was executed sucesessfully
	status = 1 => one of the AT commands was not executed sucessfully.
	status = 32 => QLTS, i.e. the network time didn't execute sucessfully.
	
	WILL MATCH FLOWCHART IN VISIO
	*/
	
	uint8_t status = 0;
	uint8_t i = 0;
	//while (i < ((sizeof(m95_connect)/(sizeof(m95_connect[0])))-0))
	while (i < 1)
	{
		if (tx_at_response(&m95_gprs[i])) {/*goto END;*/}
		//delay_ms(500);
		i++;
	}
	//if (tx_at_response(&m95_connect[i])) {status = 0; goto END;} else {at_get_radio_network_time();} //get network's time
		
	
	END: 
	
	return status;
}

uint8_t at_rf_connect(uint8_t state) {
	/*
	status = 0 => all AT commands was executed sucesessfully
	status = 1 => one of the AT commands was not executed sucessfully.
	status = 32 => QLTS, i.e. the network time didn't execute sucessfully.
	
	WILL MATCH FLOWCHART IN VISIO
	*/
	
 	uint8_t status = 0;
	uint8_t i = 0;
	uint8_t active = 1;
	
	while (active == 1) {
	switch(state) {
	
	case 0:
	i = 0;
	//while (i < ((sizeof(m95_connect)/(sizeof(m95_connect[0])))-1))
	while (i < 10)
	{
		if (tx_at_response(&m95_connect[i])) {/*goto END;*/}
		i++;
	}
	if (tx_at_response(&m95_connect[17]) == 0) {at_get_radio_network_time();} //get network's time
	state = 1;
	break;
	
	//status for open new connection
	case 1:
	//qistat: ip
	if (tx_at_response(&m95_connect[12]) == 0) {status = 0; state = 3; break;} //ip close
	if (tx_at_response(&m95_connect[10]) == 0) {status = 0; state = 3; break;} //ip initial
	if (tx_at_response(&m95_connect[11]) == 0) {status = 0; state = 3; break;} //ip status
	
	//if (tx_at_response(&m95_connect[13]) == 0) {status = 0; state = 3; break;} //already connect
	state = 2;
	break;
	
	case 2:
	//qistat: connect ok.
	if (tx_at_response(&m95_connect[14]) == 0) {status = 0; state = 4; break;} //connect ok 
	state = 1;
	break;
	
	case 3:
	if (tx_at_response(&m95_connect[15]) == 0) {} //qiopen
	if (tx_at_response(&m95_connect[16]) == 0) {} //qisrvc
	state = 4;
	break;
	
	case 4:
	active = 0; //end of connection.
	break;
	
	
	}
	}
	return status;
}

uint8_t tx(char *data, int len) {
	/*
	status = 0 => all AT commands was executed sucesessfully
	status > 0 => one of the AT commands was not executed sucessfully.
	*/
	uint8_t status = 0;
	//uint8_t i = 0;
	
	//START:
	
	//if (tx_at_response(&m95_tx[0]) == 1) {} //{status = 0; tx_at_response(&m95_reconnect[0]); tx_at_response(&m95_reconnect[1]); status = 0; goto START;} //SPECIFIED TO ELEMENT 0
	if (tx_at_response(&m95_tx[1]) == 1) {status = 0; goto END;} //SPECIFIED TO ELEMENT 0

	tx_data_response(data, len);
			
	#ifdef DEBUG
		//usart_tx_at(USART_TERMINAL, RESPONSE_FOOTER);
	#endif // DEBUG
	
	if (tx_at_response(&m95_tx[2])) {/*status = 0; goto START;*/} //SPECIFIED TO ELEMENT 0 FIX!!!!!!
	if (tx_at_response(&m95_tx[3])) {/*status = 0; goto START;*/} //SPECIFIED TO ELEMENT 0 FIX!!!!!!
	tx_at_response(&m95_disconnect[0]); //DEBUG. This might be faster since the TCP is closed anyway... no solution found why it closes this fast...
	
	END: return status;
}

// uint8_t tx_mqtt(uint8_t state) {
// 	/*
// 	status = 0 => all AT commands was executed sucesessfully
// 	status > 0 => one of the AT commands was not executed sucessfully.
// 	*/
// 	uint8_t status = 0;
// 	uint8_t i = 0;
// 	uint8_t active = 1;
// 	
// 	while (active == 1) {
// 	switch(state) {
// 	
// 	case 0:
// 	i = 0;
// 	//while (i < ((sizeof(m95_connect)/(sizeof(m95_connect[0])))-1))
// 	while (i < 10)
// 	{
// 		if (tx_at_response(&m95_connect[i])) {/*goto END;*/}
// 		i++;
// 	}
// 	if (tx_at_response(&m95_connect[17]) == 0) {at_get_radio_network_time();} //get network's time
// 	state = 1;
// 	break;
// 	
// 	//status for open new connection
// 	case 1:
// 	//qistat: ip
// 	if (tx_at_response(&m95_connect[12]) == 0) {status = 0; state = 3; break;} //ip close
// 	if (tx_at_response(&m95_connect[10]) == 0) {status = 0; state = 3; break;} //ip initial
// 	if (tx_at_response(&m95_connect[11]) == 0) {status = 0; state = 3; break;} //ip status
// 	
// 	//if (tx_at_response(&m95_connect[13]) == 0) {status = 0; state = 3; break;} //already connect
// 	state = 2;
// 	break;
// 	
// 	case 2:
// 	//qistat: connect ok.
// 	if (tx_at_response(&m95_connect[14]) == 0) {status = 0; state = 4; break;} //connect ok 
// 	state = 1;
// 	break;
// 	
// 	case 3:
// 	active = 0; //end of connection.
// 	break;
// 	}
// 	}
// 	
// 	if (tx_at_response(&m95_mqtt_connect[0]) == 1) {status = 0; goto END;} //SPECIFIED TO ELEMENT 0
// 	if (tx_at_response(&m95_mqtt_connect[1]) == 1) {status = 0; goto END;} //SPECIFIED TO ELEMENT 0
// 	if (tx_at_response(&m95_mqtt_connect[2]) == 1) {status = 0; goto END;} //SPECIFIED TO ELEMENT 0
// 	
// 	tx_data_response(0x30, 1);
// 			
// 	#ifdef DEBUG
// 		//usart_tx_at(USART_TERMINAL, RESPONSE_FOOTER);
// 	#endif // DEBUG
// 	
// 	if (tx_at_response(&m95_mqtt_connect[3]) == 1) {status = 0; goto END;} //SPECIFIED TO ELEMENT 0
// 		
// 	END: return status;
// }

uint8_t at_rf_disconnect(void) {
	/*
	status = 0 => all AT commands was executed sucesessfully
	status = 1 => one of the AT commands was not executed sucessfully.
	
	WILL MATCH FLOWCHART IN VISIO
	*/
	uint8_t status = 0;
	//uint8_t i = 0;
// 	while (i < ((sizeof(m95_disconnect)/(sizeof(m95_disconnect[0])))))
// 	{
// 		if (tx_at_response(&m95_disconnect[i])) {goto END;}
// 		i++;
// 	}
// 	
	if (tx_at_response(&m95_disconnect[1])) {goto END;} //DEBUG, since the close is already done in the tx
	
	END: return status;
}

ISR(USART0_RX_vect)
{
	*(response + response_counter) = usart_getchar(USART_RADIO);
	response_counter++;
	response_timeout_counter = 0; //reset global timeout counter for each byte read. Could ideally be lower for 20 seconds timeout commands. FIX!!!!!
}

//ISR(WDT_vect)
jalla()
{
	//sleep_disable();
	wdt_disable();
	wdt_counter++; //watchdog set at 1 second timeout
	usart_tx_at(USART_TERMINAL, RESPONSE_FOOTER); //DEBUG
		usart_putchar(USART_TERMINAL, (0x30+wdt_counter)); //DEBUG
		usart_tx_at(USART_TERMINAL, RESPONSE_FOOTER); //DEBUG	
	delay_s(2);
	wdt_reset();
	//rtc_init_period(2); //using RTC as sampler timer.
	//wdt_enable(9);
	rtc_init_period(1);
	//sleep_enable();
}

//main_function()
//ISR(TC0_vect) // 
ISR(WDT_vect)
{
	wdt_disable();
	wdt_counter++; //watchdog set at 1 second timeout
	
	#ifdef DEBUG
		usart_tx_at(USART_TERMINAL, RESPONSE_FOOTER); //DEBUG
		usart_putchar(USART_TERMINAL, (0x30+wdt_counter)); //DEBUG
		usart_tx_at(USART_TERMINAL, RESPONSE_FOOTER); //DEBUG	
	#endif // DEBUG
	
	if (wdt_counter < WAKEUP_RATE)
	{
		goto END;
	} else {wdt_counter = 0;} //reset counter if limit is reached.
	
	RTC_ISR_ACTIVE = 1;
	while (RTC_ISR_ACTIVE == 1)
	{
		
		switch(controller_state) {
			
			case READ_EXT_DATA:
				controller_next_state = MEASURE;
				break;
			
			case MEASURE:
				PRR0 &= ~(1<<PRADC); //enable adc clock
				ADCSRA |= (1<<ADEN);
				//SPECIAL MEASUREMENTS REQUIRED BY THE LOADCELL///////////////////////////
				loadcell_power_on();
				tx_data[POSITION_CURRENT] = adc_10_to_12_bits(ADC_MUX_ADC0); //NEED TO FIX ADC CONVERSINS!!!!!!!!
				//////////////////////////////////////////////////////////////////////////
				
				//GENERAL MEASUREMENTS
// 				tx_data[POSITION_ANA0] = adc_result_average(ADC_MUX_ADC0, ADC_NUM_AVG); //NEED TO FIX ADC CONVERSINS!!!!!!!!
// 				tx_data[POSITION_ANA0] = adc_result_average(ADC_MUX_1V1, ADC_NUM_AVG); //PIN CHANGE HAVE NO EFFECT ON ADCB
// 				tx_data[POSITION_ANA1] = adc_result_average(ADC_MUX_ADC1, ADC_NUM_AVG); //
// 				tx_data[POSITION_ANA2] = adc_result_average(ADC_MUX_ADC2, ADC_NUM_AVG); //
// 				tx_data[POSITION_ANA3] = adc_result_average(ADC_MUX_ADC3, ADC_NUM_AVG); //
// 				tx_data[POSITION_ANA4] = adc_result_average(ADC_MUX_ADC4, ADC_NUM_AVG); //
// 				tx_data[POSITION_ANA5] = adc_result_average(ADC_MUX_ADC5, ADC_NUM_AVG); //
// 				//tx_data[POSITION_TEMP] = adc_result_average(ADC_MUX_TEMPSENSE, 1); //PIN CHANGE HAVE NO EFFECT ON ADCB
 				//tx_data[POSITION_VDD] = adc_result_average(ADC_MUX_1V1, 1); //PIN CHANGE HAVE NO EFFECT ON ADCB
				 tx_data[POSITION_VDD] = ((1.05*1024)/(adc_read_10bit(ADC_MUX_1V1, ADC_VREF_AVCC)))*1000; //PIN CHANGE HAVE NO EFFECT ON ADCB
				
				ADCSRA &= ~(1<<ADEN);
				PRR0 |= (1<<PRADC); //disable ADC clock
				 
				//SPECIAL MEASUREMENTS REQUIRED BY THE LOADCELL///////////////////////////
				loadcell_power_off();
				//////////////////////////////////////////////////////////////////////////
				
				//SPECIAL MEASUREMENTS REQUIRED BY THE LOADCELL////////////////////////////////////////////////////////////////////////
				accu_data += tx_data[POSITION_CURRENT]; //controller_measure(9, &tx_data); //measure with averaging, and accumulate.
				loadcell_min_max_tran(tx_data[POSITION_CURRENT], &tx_data); //check if new value should be stored in min, max and tran.
				tx_data[POSITION_PREV] = tx_data[POSITION_CURRENT]; //store adc value for next measurement.
				///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				
				transmit_counter++;		
				tx_data[POSITION_TIME] = transmit_counter; //increase timestamp counter.
			
				if (tx_data[POSITION_TIME] >= (TRANSMIT_RATE/WAKEUP_RATE)) //if accumulation limit is reached.
				{
					transmit_counter = 0; //reset counter
					controller_next_state = CALC; //limit reached, go to next
					} else {
					controller_next_state = READ_EXT_DATA; //Start from top again
					RTC_ISR_ACTIVE = 0; //Break loop and go to sleep again
				}
				break;
			
			case CALC:
				//SPECIAL MEASUREMENTS REQUIRED BY THE LOADCELL////////////////////////////////////////////////////////////////////////
				tx_data[POSITION_AVG] = controller_calc_avg(accu_data, tx_data[POSITION_TIME]); //calc and store average.
				accu_data = 0; //reset parameters
				///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				controller_next_state = STORE_EXT_MEM;
				break;
			
			case STORE_EXT_MEM:
				tx_data[POSITION_TIME] = 0; //reset accumulation counter if something???????????????
				
				#ifdef DEBUG
					char myval[5] = "";
					reset_char_array(myval, 5);
					itoa(tx_data[POSITION_AVG], myval, 10);
					usart_tx_at(USART_TERMINAL, RESPONSE_HEADER);
					usart_tx_at(USART_TERMINAL, myval);
					usart_tx_at(USART_TERMINAL, RESPONSE_FOOTER);
				#endif // DEBUG
								
				controller_next_state = RF_POWER_ON;
				break;
			
			case RF_POWER_ON:
				if (radio_power_on() == 1) //power on and check if it fails
 				{
					
// 					tx_data[POSITION_STATUS] |= (1<<STATUS_BIT_RF_POWER_ON); //set failure status
// 					controller_next_state = RF_POWER_OFF; //if failure go to power off
// 					break;
 				}
				controller_next_state = RF_CONNECT;
				break;
			
			case RF_CONNECT: //NEED MORE POWER!!!!!!
				if (at_rf_status() != 0)
				{
					controller_next_state = RF_POWER_OFF; // RF_DISCONNECT; //if failure go to disconnect
					break;
				}
				 
				at_rf_gprs();
				if (at_rf_connect(0) != 0) //Connect to network. MAKE STATUS REPORT FROM THIS!!!!!!!
				{
					tx_data[POSITION_STATUS] |= (1<<STATUS_BIT_RF_CONNECT); //set failure status
					controller_next_state = RF_POWER_OFF; // RF_DISCONNECT; //if failure go to disconnect
					break;
				}
				controller_next_state = GENERATE_PACKAGE;
				break;
			
			case GENERATE_PACKAGE:
				transfer_data_package_counter = 2;
				while (transfer_data_package_counter < 8)
 				{
					data_to_char(&tx_data[transfer_data_package_counter], 1, &tx_data_bytes, TRANSFER_DATA_BASE); //data to ascii
					itoa(transfer_data_package_counter, mqtt_sub_topic, 10); //counter to text for sub-topic
					transfer_data_length_package = mqtt_packet(&tx_data_bytes, &tx_data_package, TRANSFER_DATA_SIZE_PACKAGE, &mqtt_sub_topic); //convert ascii data to MQTT package.
								
					if (transfer_data_package_counter > 2)
					{
						at_rf_connect(1); ////status for open new connection
					}
					
					tx(&tx_data_package, transfer_data_length_package); //transmit package. GENERATE STATUS FROM THIS.
					reset_char_array(&mqtt_sub_topic, 3);
					
					//tx_mqtt(0); //transmit package. GENERATE STATUS FROM THIS.
					
					#ifdef DEBUG //output package size
// 						char package_lenght[5] = "";
// 						char mystring[5] = "";
// 						itoa(transfer_data_length_package, package_lenght, 10);
// 						strcpy(mystring, package_lenght);
// 						usart_tx_at(USART_TERMINAL, mystring);
					#endif // DEBUG

					transfer_data_package_counter++;
					
				}
				controller_next_state = TX_DATA;
				break;
			
			case TX_DATA:
				controller_next_state = RX_DATA;
				break;
			
			case RX_DATA:
				controller_next_state = RF_DISCONNECT;
				break;
			
			case RF_DISCONNECT:
 				if (at_rf_disconnect() != 0) //Status will not be transmitted, but could probably be stored for later.
 				{
// 					//tx_data[POSITION_STATUS] |= (1<<STATUS_BIT_RF_DISNNECT); //set failure status
// 					controller_next_state = RF_POWER_OFF; // RF_DISCONNECT; //if failure go to disconnect
// 					break;
 				}
				controller_next_state = RF_POWER_OFF;
				break;
			
			case RF_POWER_OFF:
				radio_power_off_at(); //radio power down
				controller_next_state = RESET_REGISTERS;
				break;
			
			case RESET_REGISTERS:
				reset_all_data(); //reset all arrays
				controller_next_state = READ_EXT_DATA;
				RTC_ISR_ACTIVE = 0; //FINISHED, break LOOP!
				break;
			
			default:
				controller_next_state = RESET_REGISTERS; //TRY RESET.
				break;
		}
		
		controller_state = controller_next_state; //NEXT STATE => STATE
		
	}

	END:
	
	wdt_reset();
	//wdt_enable();
	rtc_init_period(1);
	return;
	
}

/*! \brief Main function.
 */
int main(void)
{
	//disables interrupts
	cli();
	
	//sysclk_init();
	PRR0 |= (1<<PRTWI) | (1<<PRTIM2) | (1<<PRTIM0) | (1<<PRTIM1) | (1<<PRSPI) | (1<<PRADC);
// 	MCUCR = 0x60;
// 	MCUCR = 0x40;
		
	//USART
	usart_init_rs232(USART_RADIO, &USART_OPTIONS); //Radio UART
	//sysclk_enable_module(POWER_RED_REG0, PRUSART0_bm);
	PRR0 &= ~(1<<PRUSART0);
	
	#ifdef DEBUG
		usart_init_rs232(USART_TERMINAL, &USART_OPTIONS); //Radio UART
		//sysclk_enable_module(POWER_RED_REG0, PRUSART1_bm);
		PRR0 &= ~(1<<PRUSART1);
		usart_putchar(USART_TERMINAL, 0x40);
		usart_putchar(USART_TERMINAL, 0x41);
	#endif // DEBUG
	
	//initialise loadcell pins
	loadcell_pins_init();

	//ADC setup
	adc_initialization();

 	//initialize radio pins
 	delay_ms(100); //wait for voltages to settle
 	radio_pins_init();
 	delay_ms(100);

	//Shut down radio, might be on
	//FIND A SOLUTION THAT ALWAYS GUARANTEE POWER OFF OR POWER SLEEP, OR WHATEVER IS THE BEST OPTION.
	if( (STATUS_PORT & (1<<STATUS_PIN)) == 0x20 ) //have a counter and reset if fail
	{
		START:
		if(radio_power_off_at() == 1)
		{
			radio_power_off();
		}
		
		if( (STATUS_PORT & (1<<STATUS_PIN)) == 0x20 ) //if it won't turn off try again...
		{
			goto START;
		}
	}
	
	//reset all tx data and date
	reset_all_data(); //PROBLEM FIX!!!!!!!!!!!!!!!!!!!!!!
	
	
	rtc_init_period(2); //using RTC as sampler timer.
 	//wdt_reset();
 	//wdt_enable(WDTO_4S);
 	
	//Set sleep mode
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	//set_sleep_mode(SLEEP_MODE_PWR_SAVE);
	
	sei(); //enable interrupts
		
	//go to sleep and let interrupts do the work...zzz....zzzz
	while (1)
	{
//		sleep_mode();
 		sleep_enable();
 		sleep_bod_disable();
		 //sei();
 		sleep_cpu();
 		sleep_disable(); 	
		//sleep_cpu();
		/*sleepmgr_enter_sleep();*/
		//sleep_enable();
// 		delay_s(1);
// 		main_function();
		
	}
		
}