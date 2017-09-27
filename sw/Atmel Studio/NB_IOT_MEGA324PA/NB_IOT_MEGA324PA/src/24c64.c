#include <avr/io.h>
#include <util/delay.h>

#include "24c64.h"

void EEOpen()
{
	//Set up TWI Module
	TWBR = 1; //5; //Bit rate generator division factor
	TWSR &= (~((1<<TWPS1)|(1<<TWPS0))); //VERIFY AGAINST FCPU AND MEMORY SPEED!!!!!
	//At 1MHz and 1 division factor and 1 in prescaler the clock frequency is approx 55kHz. Pull up resistor must be >1k.

}

uint8_t EEWriteByte(uint16_t address,uint8_t data)
{
	do
	{
		//Put Start Condition on TWI Bus
		TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

		//Poll Till Done
		while(!(TWCR & (1<<TWINT)));

		//Check status
		if((TWSR & 0xF8) != 0x08)
			return FALSE;

		//Now write SLA+W
		//EEPROM @ 00h
		TWDR=0b10100000;	

		//Initiate Transfer
		TWCR=(1<<TWINT)|(1<<TWEN);

		//Poll Till Done
		while(!(TWCR & (1<<TWINT)));
	
	}while((TWSR & 0xF8) != 0x18);
		
	//disabled FOR 256B memory
// 	//Now write ADDRH
// 	TWDR=(address>>8); 
// 
// 	//Initiate Transfer
// 	TWCR=(1<<TWINT)|(1<<TWEN);
// 
// 	//Poll Till Done
// 	while(!(TWCR & (1<<TWINT)));
// 
// 	//Check status
// 	if((TWSR & 0xF8) != 0x28)
// 		return FALSE;
// 
	//Now write ADDRL
	TWDR=(address);

	//Initiate Transfer
	TWCR=(1<<TWINT)|(1<<TWEN);

	//Poll Till Done
	while(!(TWCR & (1<<TWINT)));

	//Check status
	if((TWSR & 0xF8) != 0x28)
		return FALSE;

	//Now write DATA
	TWDR=(data);

	//Initiate Transfer
	TWCR=(1<<TWINT)|(1<<TWEN);

	//Poll Till Done
	while(!(TWCR & (1<<TWINT)));

	//Check status
	if((TWSR & 0xF8) != 0x28)
		return FALSE;

	//Put Stop Condition on bus
	TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	
	//Wait for STOP to finish
	while(TWCR & (1<<TWSTO));

	//Wait untill Writing is complete
	_delay_ms(6); //The datasheet had 5ms at max write cycle time.

	//Return TRUE
	return TRUE;

}

uint8_t EEReadByte(uint16_t address)
{
	uint8_t data;

	//Initiate a Dummy Write Sequence to start Random Read
	do
	{
		//Put Start Condition on TWI Bus
		TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

		//Poll Till Done
		while(!(TWCR & (1<<TWINT)));

		//Check status
		if((TWSR & 0xF8) != 0x08)
			return FALSE;

		//Now write SLA+W
		//EEPROM @ 00h
		TWDR=0b10100000;	

		//Initiate Transfer
		TWCR=(1<<TWINT)|(1<<TWEN);

		//Poll Till Done
		while(!(TWCR & (1<<TWINT)));
	
	}while((TWSR & 0xF8) != 0x18);
		

	//disabled FOR 256B memory
// 	//Now write ADDRH
// 	TWDR=(address>>8);
// 
// 	//Initiate Transfer
// 	TWCR=(1<<TWINT)|(1<<TWEN);
// 
// 	//Poll Till Done
// 	while(!(TWCR & (1<<TWINT)));
// 
// 	//Check status
// 	if((TWSR & 0xF8) != 0x28)
// 		return FALSE;

	//Now write ADDRL
	TWDR=(address);

	//Initiate Transfer
	TWCR=(1<<TWINT)|(1<<TWEN);

	//Poll Till Done
	while(!(TWCR & (1<<TWINT)));

	//Check status
	if((TWSR & 0xF8) != 0x28)
		return FALSE;

	//*************************DUMMY WRITE SEQUENCE END **********************


	
	//Put Start Condition on TWI Bus
	TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

	//Poll Till Done
	while(!(TWCR & (1<<TWINT)));

	//Check status
	if((TWSR & 0xF8) != 0x10)
		return FALSE;

	//Now write SLA+R
	//EEPROM @ 00h
	TWDR=0b10100001;	

	//Initiate Transfer
	TWCR=(1<<TWINT)|(1<<TWEN);

	//Poll Till Done
	while(!(TWCR & (1<<TWINT)));

	//Check status
	if((TWSR & 0xF8) != 0x40)
		return FALSE;

	//Now enable Reception of data by clearing TWINT
	TWCR=(1<<TWINT)|(1<<TWEN);

	//Wait till done
	while(!(TWCR & (1<<TWINT)));

	//Check status
	if((TWSR & 0xF8) != 0x58)
		return FALSE;

	//Read the data
	data=TWDR;

	//Put Stop Condition on bus
	TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	
	//Wait for STOP to finish
	while(TWCR & (1<<TWSTO));

	//Return TRUE
	return data;
}


		


