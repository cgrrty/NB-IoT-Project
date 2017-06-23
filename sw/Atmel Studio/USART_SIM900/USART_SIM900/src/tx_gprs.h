/*
 * tx_gprs.h
 *
 * Created: 15/05/2017 15:56:17
 *  Author: jan.rune.herheim
 */ 


#ifndef TX_GPRS_H_
#define TX_GPRS_H_

typedef enum {
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




#endif /* TX_GPRS_H_ */