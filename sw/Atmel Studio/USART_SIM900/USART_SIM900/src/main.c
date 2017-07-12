/**
 /*
 */
#include "main.h"

/*
	TODO:
	- HW data flow
	*/


/*
Data shared between the ISR and your main program must be both volatile and global in scope in the C language. 
Without the volatile keyword, the compiler may optimize out accesses to a variable you update in an ISR,
as the C language itself has no concept of different execution threads. Take the following example:
*/

//DEFINE OF THE SYSTEM CLOCK SPEED
//#define F_CPU 2000000UL

//DFEINITIOINS OF CONTROLLER STATES. THESE WILL MATCH WITH VISIO FLOW CHART.
typedef enum {
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
//////////////////////////////////////////////////////////////////////////

//DEFINITIONS OF THE AMOUNT OF EXTERNAL DATA TO BE RECEIVED
#define EXT_DATA_NR_BYTES 1 //number of bytes to receive
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
volatile static uint16_t tx_data_reset_values[TX_DATA_SIZE] = {RESET_VALUE_ANA0, RESET_VALUE_ANA1, RESET_VALUE_ANA2, RESET_VALUE_ANA3,
	RESET_VALUE_ANA4, RESET_VALUE_ANA5, RESET_VALUE_TEMP, RESET_VALUE_VDD, RESET_VALUE_DIO, RESET_VALUE_TIME,
	RESET_VALUE_YEAR, RESET_VALUE_MONTH, RESET_VALUE_DAY, RESET_VALUE_HOUR, RESET_VALUE_MINUTE, RESET_VALUE_SECOND, RESET_VALUE_STATUS}; //declare the reset register.
//////////////////////////////////////////////////////////////////////////


//DEFINITIONS OF PINS CONNECTED TO THE RADIO
#define PWRKEY_PORT PORTE 
#define PWRKEY_PIN 6
#define STATUS_PORT PORTE
#define STATUS_PIN 7
#define NETLIGHT_PORT PORTR
#define NETLIGHT_PIN 0
//////////////////////////////////////////////////////////////////////////

//DEFINITIONS OF THE EXTERNAL DATA PINS
#define REQUEST_DATA_PORT PORTF
#define REQUEST_DATA_PIN 4
//////////////////////////////////////////////////////////////////////////

//SPECIALIZED PARAMETERS FOR THE LOAD CELL APPLICATION
//local data declarations
uint32_t accu_data = 0; //allocating internal accumulation storage.
//////////////////////////////////////////////////////////////////////////

//RADIO RESPONSE ARRAY AND SIZE DEFINITIONS
#define RESPONSE_SIZE 128
volatile char response[RESPONSE_SIZE];
volatile uint8_t response_counter = 0;
volatile uint32_t response_timeout = 0;
volatile uint32_t response_timeout_counter = 0;

volatile uint8_t *tx_char = 0;
//////////////////////////////////////////////////////////////////////////

//DEFINITIONS OF STATUS AND MODES PARAMETERS
volatile uint8_t RTC_ISR_ACTIVE = 0;
//////////////////////////////////////////////////////////////////////////

//INTERRUPT DEFINITIONS
#define INT_LEVEL_LOW 0x01
//////////////////////////////////////////////////////////////////////////

//TIMER DEFINITIONS
// #define AT_TIMEOUT_TC TCC0 //define AT command timeout counter.
// #define AT_REPEAT 9 //number of times to repeat an AT command.

#define CONFIG_RTC_PRESCALER RTC_PRESCALER_DIV1024_gc
#define CONFIG_RTC_SOURCE SYSCLK_RTCSRC_ULP
//////////////////////////////////////////////////////////////////////////

//ADC definitions
#define ADC_NUM_AVG 9 //number of averages
//////////////////////////////////////////////////////////////////////////

//DEFINITIONS OF DATA FORMATS AND DATA SIZES TO BE TRANSMITTED THROUGH THE RADIO
#define TRANSFER_DATA_BASE 10 //DATA FORMAT THE DATA IS TRANSFERRED WITH. 10 = Base10 (decimal), 16 = Base16 (hex), 32 = Base32
#define TX_DATA_DIGITS 4 //NUMBER OF DATA DIGITS TO TRANSMIT: 2 or 4
#define TX_DATE_DIGITS 2 //Number of date digits to transmit: 1 or 2
#define TX_ASCII 1 //0 will transfer hex bytes, 1 will transfer ascii coded bytes: 1 or 2

#define TRANSFER_DATA_SIZE 128 //ASSUMING 128 bytes are enough.....
volatile char tx_data_bytes[TRANSFER_DATA_SIZE] = ""; //Final data to be transmitted (and stored?)
#define TRANSFER_DATA_SIZE_PACKAGE (TRANSFER_DATA_SIZE*2) //ASSUMING 2 TIMES THE DATA SIZE IS THE TOTAL PACKAGE OVERHEAD.
volatile char tx_data_package[TRANSFER_DATA_SIZE_PACKAGE] = ""; //Final data package generated from the tx_data_bytes array.
int transfer_data_length_package = 0; //ACTUAL PACKAGE SIZE TO BE TRANSMITTED. CALCULATED IN PROGRAM. KEEP AS LOW AS POSSIBLE!!!!!!
////////////////////////////////////////////////////////////////////////////////////

//Functions related to the radio communication
void usart_tx_at(USART_t *usart, uint8_t *cmd) {
	
	while(*cmd) {
		usart_putchar(usart, *cmd++); //send the command
	}
	
}

void usart_tx_char(USART_t *usart, char *cmd) {
	
	//send the command
	while(*cmd) {
		usart_putchar(usart, *cmd++);
	}
	
}

#ifdef DEBUG
void led_blink(uint16_t on_time) {
	
	PORTQ.OUT &= ~(1<<3);
	delay_ms(on_time);
	PORTQ.OUT |= (1<<3);
	delay_ms(on_time);
}

void led_blink2(uint32_t on_time) {
	volatile uint16_t i = 0;
	
	PORTQ.OUT &= ~(1<<3);
	while (i < on_time)
	{
		delay_ms(1);
		i++;
	}
	i = 0;
	PORTQ.OUT |= (1<<3);
	while (i < on_time)
	{
		delay_ms(1);
		i++;
	}
}
#endif // DEBUG

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

static void adc_init(void)
{
	//ADCA configurations
	struct adc_config adca_conf;
	struct adc_channel_config adca_ch_conf;
	
	adc_read_configuration(&ADCA, &adca_conf);
	adc_set_conversion_parameters(&adca_conf, ADC_SIGN_OFF, ADC_RES_12, ADC_REF_VCC); //Single ended, 12 bits, vref = vdd/1,6 ~ 2V @ 3,3V.
	adc_set_conversion_trigger(&adca_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_set_clock_rate(&adca_conf, 200000UL); //200kHz clock
	adc_write_configuration(&ADCA, &adca_conf);
	
	adcch_read_configuration(&ADCA, ADC_CH0, &adca_ch_conf);
	adcch_set_input(&adca_ch_conf, ADCCH_POS_PIN0, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&ADCA, ADC_CH0, &adca_ch_conf); //Channel 0
	//////////////////////////////////////////////////////////////////////////
	
	//ADCB configurations
	struct adc_config adcb_conf;
	struct adc_channel_config adcb_ch_conf;
	
	adc_read_configuration(&ADCB, &adcb_conf);
	adc_set_conversion_parameters(&adcb_conf, ADC_SIGN_OFF, ADC_RES_12, ADC_REF_VCC); //Single ended, 12 bits, vref = vdd/1,6 ~ 2V @ 3,3V.
	adc_set_conversion_trigger(&adcb_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_set_clock_rate(&adcb_conf, 200000UL); //200kHz clock
	adc_write_configuration(&ADCB, &adcb_conf);
	
	adcch_read_configuration(&ADCB, ADC_CH0, &adcb_ch_conf);
	adcch_set_input(&adcb_ch_conf, ADCCH_POS_BANDGAP, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&ADCB, ADC_CH0, &adcb_ch_conf); //Channel 0
	
	adcch_set_input(&adcb_ch_conf, ADCCH_POS_TEMPSENSE, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&ADCB, ADC_CH1, &adcb_ch_conf); //Channel 1
	//////////////////////////////////////////////////////////////////////////
}

uint16_t adc_result_single(ADC_t *adc, uint8_t ch_mask) {
	adc_start_conversion(adc, ch_mask);
	adc_wait_for_interrupt_flag(adc, ch_mask);
	return adc_get_result(adc, ch_mask);
}

uint16_t adc_result_average (ADC_t *adc, uint8_t adc_ch ,uint8_t adc_pin, uint8_t num_avg) {
	
	uint8_t i = 0;
	uint32_t res = 0;
	uint16_t res_median[num_avg];
	
	if (adc == &ADCA)
	{
		adc->CH0.MUXCTRL = (adc_pin << 3) & (0xff);
	}
	
	
	while (i<num_avg)
	{
		adc_start_conversion(adc, adc_ch);
		adc_wait_for_interrupt_flag(adc, adc_ch);

		res_median[i] = adc_get_result(adc, adc_ch);
		res = res + res_median[i];
		i++;
	}
	
	res = res/num_avg;
	
	
	//return res_median[(num_avg-1)/2];
	return res;	
}

uint16_t controller_calc_avg(uint32_t data, uint16_t cnt) {
	 uint16_t avg = 0;
	 
	 avg = data/cnt;
	 return avg;
}

uint8_t data_to_char(uint16_t *array_data, uint8_t array_data_len, char *array_ascii, int base) {
	uint8_t status = 0;
	uint8_t i = 0;
	uint8_t j = 0;
	char temp[5] = ""; //MAX 4 VALUES + NULL TERMINATION
		
	//CONVERT ALL 2 BYTES NUMBERS
	while (i <= POSITION_TIME)
	{
		j=1;
		while (TX_DATA_DIGITS-j > 0)
		{
			if ((*(array_data+i) < pow(base,TX_DATA_DIGITS-j))) //CHECK IF NUMBER IS LESS THAN LIMITS
			{
				strcat(array_ascii, "0"); //ADD LEADING ZEROS
			}
					
			j++;
		}
		
		if (TX_ASCII)
		{
			itoa(*(array_data+i), temp, base); //CONVERT NUMBER TO ASCII
		} else {
			temp[0] = (*(array_data+i) >> 8) & 0xff; //JUST GRAB THE BYTES
			temp[1] = *(array_data+i) & 0xff;
		}
				
		strcat(array_ascii, temp); //APPEND NUMBER
		strcat(array_ascii, ","); //DEBUG
		reset_char_array(&temp, sizeof(temp));
		i++;
	}
	//////////////////////////////////////////////////////////////////////////
		
	//CONVERT ALL 1 BYTES NUMBERS
	i = POSITION_YEAR;
	while (i <= POSITION_STATUS)
	{
		j=1;
		while (TX_DATE_DIGITS-j > 0)
		{
			if ((*(array_data+i) < pow(base,TX_DATE_DIGITS-j))) //CHECK IF NUMBER IS LESS THAN LIMITS
			{
				strcat(array_ascii, "0"); //ADD LEADING ZEROS
			}
			j++;
		}
		
		if (TX_ASCII)
		{
			itoa(*(array_data+i), temp, base); //CONVERT NUMBER TO ASCII
		} else {
			temp[0] = *(array_data+i) & 0xff; //JUST GRAB THE BYTES
		}
		
		strcat(array_ascii, temp); //APPEND NUMBER
		strcat(array_ascii, ","); //DEBUG
		reset_char_array(&temp, sizeof(temp));
		i++;
	}
	//////////////////////////////////////////////////////////////////////////
	return status;
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

void ext_data_pins_init() {
	//Set up port and pin to wake and request data from external source
	REQUEST_DATA_PORT.DIR |= (1<<REQUEST_DATA_PIN); //request pin as output
	
	//SETUP CTS AND RTS LATER!!!!!!	
	
}

uint8_t radio_power_on(void) {
	uint8_t status = 0;
	uint8_t cnt_pwron = 0;
	
	PWRKEY_PORT.OUT &= ~(1<<PWRKEY_PIN); //reset
	delay_ms(1); //wait for battery voltage to settle.
	PWRKEY_PORT.OUT |= (1<<PWRKEY_PIN); //reset of radio
	delay_ms(100); //boot time, 100ms recommended for m95
	PWRKEY_PORT.OUT &= ~(1<<PWRKEY_PIN);
	delay_ms(800); //time before m95 is running. There exist a status bit that might be useful to monitor.
	delay_ms(400);
	PWRKEY_PORT.OUT |= (1<<PWRKEY_PIN); //normal level for this pin
	
	//wait for status
	while ( (!(STATUS_PORT.IN & (1<<STATUS_PIN))) & (cnt_pwron < AT_REPEAT_LONG) )
	{
		delay_ms(300);
		cnt_pwron++;
	} 
	
	if (cnt_pwron == AT_REPEAT_LONG)
	{
		status = 1;
	}
	
	return status;
}

uint8_t radio_power_down(void) {
	uint8_t status = 0;
	uint8_t cnt_pwrdwn = 0;
	
	//power down
	PWRKEY_PORT.OUT &= ~(1<<PWRKEY_PIN);
	delay_s(1);
	PWRKEY_PORT.OUT |= (1<<PWRKEY_PIN); 
	delay_s(1);
	PWRKEY_PORT.OUT &= ~(1<<PWRKEY_PIN);
	
	while ( (STATUS_PORT.IN & (1<<STATUS_PIN)) & (cnt_pwrdwn < AT_REPEAT_LONG) )
	{
		delay_ms(300);
		cnt_pwrdwn++;
	}
	
	
	if (cnt_pwrdwn == AT_REPEAT_LONG)
	{
		status = 1;
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
	reset_tx_data(&tx_data, &tx_data_reset_values, TX_DATA_SIZE);
	reset_char_array(&tx_data_bytes, TRANSFER_DATA_SIZE);
	reset_char_array(&tx_data_package, TRANSFER_DATA_SIZE_PACKAGE);
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
	uint32_t i = 0;
	
	ret = 0;
	while (tx_at_cnt < opt->retries) //Less than nr of retries to send the AT command
	{
		reset_char_array(&response, RESPONSE_SIZE); //reset response buffer
		response_counter = 0; //RESET COUNTER
		response_timeout = opt->resp_time;
		response_timeout_counter = 0;
		
		usart_tx_at(USART_SERIAL_SIM900, opt->cmd); //send AT command to radio
		
		usart_set_rx_interrupt_level(USART_SERIAL_SIM900, USART_INT_LVL_MED); //READY FOR RECEIVING BYTES
		while (response_timeout_counter < response_timeout) {
			response_timeout_counter++;
			delay_us(1);
			ret = strstr(response, opt->comp); //DO THE COMPARISON AND BREAK THE LOOP
			if (ret != 0) //correct response received. IDEALLY IT SHOULD CHECK FOR WRONG RESPONSES TO AVOID TIMOUT TO BE RUN IF IT HAPPENS
			{
				status = 0;
				goto END;
				} else {
				status = 1;
			}
		}
		
		delay_ms(300);
		tx_at_cnt++;
	}
	
	END:
	usart_set_rx_interrupt_level(USART_SERIAL_SIM900, USART_INT_LVL_OFF); //disable rx interrupts
	
	#ifdef DEBUG
		usart_tx_at(USART_SERIAL_EXAMPLE, response); //DEBUG
		//usart_tx_at(USART_SERIAL_EXAMPLE, RESPONSE_FOOTER); //DEBUG
	#endif // DEBUG
	
	return status;
}

uint8_t at_rf_connect(void) {
	/*
	status = 0 => all AT commands was executed sucesessfully
	status = 1 => one of the AT commands was not executed sucessfully.
	status = 32 => QLTS, i.e. the network time didn't execute sucessfully.
	
	WILL MATCH FLOWCHART IN VISIO
	*/
	
	uint8_t status = 0;
	uint8_t i = 0;
	while (i < ((sizeof(m95_connect)/(sizeof(m95_connect[0])))-1))
	{
		if (tx_at_response(&m95_connect[i])) {/*goto END;*/}
		i++;
	}
	if (tx_at_response(&m95_connect[i])) {status = 32; goto END;} else {at_get_radio_network_time();} //get network's time
	
	END: 
	
	return status;
}

uint8_t at_rf_disconnect(void) {
	/*
	status = 0 => all AT commands was executed sucesessfully
	status = 1 => one of the AT commands was not executed sucessfully.
	
	WILL MATCH FLOWCHART IN VISIO
	*/
	uint8_t status = 0;
	uint8_t i = 0;
	while (i < ((sizeof(m95_disconnect)/(sizeof(m95_disconnect[0])))))
	{
		if (tx_at_response(&m95_disconnect[i])) {/*goto END;*/}
		i++;
	}
	
	END: return status;
}

uint8_t tx(char *data, int len) {
	/*
	status = 0 => all AT commands was executed sucesessfully
	status > 0 => one of the AT commands was not executed sucessfully.
	*/
	uint8_t status = 0;
	uint8_t i = 0;
		
	if (tx_at_response(&m95_tx[0])) {/*status = 1; goto END;*/} //SPECIFIED TO ELEMENT 0
	if (tx_at_response(&m95_tx[1])) {/*status = 1; goto END;*/} //SPECIFIED TO ELEMENT 0
	while (i < len)
	{
		//usart_putchar(USART_SERIAL_SIM900, *(data+i));
		#ifdef DEBUG
			usart_putchar(USART_SERIAL_EXAMPLE, *(data+i)); //DEBUG
		#endif // DEBUG
		i++;
	}
	usart_tx_at(USART_SERIAL_SIM900, CTRL_Z);
	//if (tx_at_response(&m95_tx[2])) {/*status = 1; goto END;*/} //SPECIFIED TO ELEMENT 0 FIX!!!!!!
			
	END: return status;
}


ISR(USARTC0_RXC_vect)
{
	*(response + response_counter) = usart_getchar(USART_SERIAL_SIM900);
	response_counter++;
	response_timeout_counter = 0; //reset global timeout counter for each byte read. Could ideally be lower for 20 seconds timeout commands. FIX!!!!!
}

ISR(RTC_OVF_vect)
{
	sysclk_disable_module(SYSCLK_PORT_GEN, SYSCLK_RTC);
	//cli(); //disable interrupts. Other way of disabling and resetting?
	#ifdef DEBUG
		led_blink(50); //DEBUG
	#endif // DEBUG
	
	RTC_ISR_ACTIVE = 1;
	while (RTC_ISR_ACTIVE == 1)
	{
		
		switch(controller_state) {
			
			case READ_EXT_DATA:
				reset_char_array(&response, RESPONSE_SIZE); //reset response buffer
				REQUEST_DATA_PORT.OUTSET |= (1<<REQUEST_DATA_PIN); //set signal high
				//at_response(USART_EXT_DATA, RESPONSE_TIME_300M, &response); //read the response from the radio
				REQUEST_DATA_PORT.OUTSET &= ~(1<<REQUEST_DATA_PIN); //set signal low
				#ifdef DEBUG
					usart_tx_at(USART_SERIAL_EXAMPLE, response);
				#endif // _DEBUG
				uint16_t ext_data = (response[0] << 8) | response[1]; //convert response to bytes and store in data registers
				//This position needs to be specified for each use case dependent on available registers.
				tx_data[POSITION_DIO] = ext_data;
				//////////////////////////////////////////////////////////////////////////
				controller_next_state = MEASURE;
				//controller_next_state = RESET_REGISTERS; //DEBUG
				break;
			
			case MEASURE:
				//GENERAL MEASUREMENTS
 				tx_data[POSITION_ANA0] = adc_result_average(&ADCA, ADC_CH0, ADCCH_POS_PIN0, ADC_NUM_AVG); //
// 				tx_data[POSITION_ANA1] = adc_result_average(&ADCA, ADC_CH0, ADCCH_POS_PIN1, ADC_NUM_AVG); //
// 				tx_data[POSITION_ANA2] = adc_result_average(&ADCA, ADC_CH0, ADCCH_POS_PIN2, ADC_NUM_AVG); //
// 				tx_data[POSITION_ANA3] = adc_result_average(&ADCA, ADC_CH0, ADCCH_POS_PIN3, ADC_NUM_AVG); //
// 				tx_data[POSITION_ANA4] = adc_result_average(&ADCA, ADC_CH0, ADCCH_POS_PIN4, ADC_NUM_AVG); //
// 				tx_data[POSITION_ANA5] = adc_result_average(&ADCA, ADC_CH0, ADCCH_POS_PIN5, ADC_NUM_AVG); //
				tx_data[POSITION_TEMP] = adc_result_average(&ADCB, ADC_CH0, ADCCH_POS_TEMPSENSE, 1); //PIN CHANGE HAVE NO EFFECT ON ADCB
				tx_data[POSITION_VDD] = adc_result_average(&ADCB, ADC_CH1, ADCCH_POS_BANDGAP, 1); //PIN CHANGE HAVE NO EFFECT ON ADCB
				
				//SPECIAL MEASUREMENTS REQUIRED BY THE LOADCELL////////////////////////////////////////////////////////////////////////
				accu_data += tx_data[POSITION_CURRENT]; //controller_measure(9, &tx_data); //measure with averaging, and accumulate.
				loadcell_min_max_tran(tx_data[POSITION_CURRENT], &tx_data); //check if new value should be stored in min, max and tran.
				tx_data[POSITION_PREV] = tx_data[POSITION_CURRENT]; //store adc value for next measurement.
				///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				
				tx_data[POSITION_TIME]++; //increase timestamp counter.
								
				if (tx_data[POSITION_TIME] >= (TRANSMIT_RATE/WAKEUP_RATE)) //if accumulation limit is reached.
				{
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
				controller_next_state = RF_POWER_ON;
				break;
			
			case RF_POWER_ON:
				if (radio_power_on() == 1) //power on and check if it fails
				{
					tx_data[POSITION_STATUS] |= (1<<STATUS_BIT_RF_POWER_ON); //set failure status
					controller_next_state = RF_POWER_OFF; //if failure go to power off
					break;
				}
				controller_next_state = RF_CONNECT;
				break;
				
			case RF_CONNECT:
				if (at_rf_connect() != 0) //Connect to network. MAKE STATUS REPORT FROM THIS!!!!!!!
				{
					tx_data[POSITION_STATUS] |= (1<<STATUS_BIT_RF_CONNECT); //set failure status
					controller_next_state = RF_DISCONNECT; //if failure go to disconnect
					break;
 				}
				controller_next_state = GENERATE_PACKAGE;
				break;
				
			case GENERATE_PACKAGE:
				data_to_char(&tx_data, TX_DATA_SIZE, &tx_data_bytes, TRANSFER_DATA_BASE);
 				transfer_data_length_package = mqtt_packet(&tx_data_bytes, &tx_data_package, TRANSFER_DATA_SIZE_PACKAGE); //convert ascii data to MQTT package.
				#ifdef DEBUG //output package size
					char package_lenght[5] = "";
					char mystring[5] = "";
					itoa(transfer_data_length_package, package_lenght, 10);
					strcpy(mystring, package_lenght);
					usart_tx_at(USART_SERIAL_EXAMPLE, mystring);
				#endif // DEBUG
				controller_next_state = TX_DATA;
				break;
				
			case TX_DATA:
				tx(&tx_data_package, transfer_data_length_package); //transmit package. GENERATE STATUS FROM THIS.
				controller_next_state = RX_DATA;		
				break;
			
			case RX_DATA:
				controller_next_state = RF_DISCONNECT;		
				break;
				
			case RF_DISCONNECT:
				at_rf_disconnect(); //Disconnect
				controller_next_state = RF_POWER_OFF;
				break;
			
			case RF_POWER_OFF:
				radio_power_down(); //radio power down
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

	
	RTC.CNT = 0; //RESET RTC COUNTER.
	sysclk_enable_module(SYSCLK_PORT_GEN, SYSCLK_RTC);
	//sei(); //enable interrupt, go to sleep
}

/*! \brief Main function.
 */
int main(void)
{
	//disables interrupts
	cli();
		
	/* Initialize the board.
	 * The board-specific conf_board.h file contains the configuration of
	 * the board initialization.
	 */
	board_init();
	pmic_init(); 
	sysclk_init(); //disables all peripheral clocks
	
	//select system clock
	//CLK.CTRL = 0x01; //2M
	
	#ifdef DEBUG
		//LED setup
		PORTQ.DIR |= (1<<3);
		PORTQ.OUT |= (1<<3);
	#endif // DEBUG
	
	
	
	//ADC setup
	adc_init();
	adc_enable(&ADCA); //Later??? By interrupt?
	adc_enable(&ADCB); //Later??? By interrupt?
		
	#ifdef DEBUG //terminal communication
		usart_init_rs232(USART_SERIAL_EXAMPLE, &USART_SERIAL_OPTIONS);
		sysclk_enable_module(SYSCLK_PORT_E, 4);
	#endif // DEBUG
	
	usart_init_rs232(USART_SERIAL_SIM900, &USART_SERIAL_OPTIONS); //Radio UART
	//usart_set_rx_interrupt_level(USART_SERIAL_SIM900, USART_INT_LVL_MED);
	sysclk_enable_module(SYSCLK_PORT_C, 4);
	
	usart_init_rs232(USART_EXT_DATA, &USART_SERIAL_OPTIONS); //External data UART
	sysclk_enable_module(SYSCLK_PORT_F, 4);
	
	//Initialize external data pins
	ext_data_pins_init();
	
	//initialize radio pins
	radio_pins_init();
	delay_s(1);
	
	//Shut down radio if already awake
	//check if radio is off, and turn of if it's on
	if (STATUS_PORT.IN & (1<<STATUS_PIN))
	{
		radio_power_down();
	}
	////////////////////////////////////////////////////////
		
	//WDT setup for interrupt
	//////////////////////////////////////////////////////////////////////////
		
	//reset all tx data and date
	reset_all_data();
	
	//RTC setup.
	PR.PRGEN &= ~(1<<2); //enable the RTC clock
	sleepmgr_init();
	rtc_init_period(WAKEUP_RATE); //using RTC as sampler timer.
	
	sei(); //enable interrupts
	
	
	
	//go to sleep and let interrupts do the work...zzz....zzzz
	while (1)
	{
		sleepmgr_enter_sleep();
	}
		
}