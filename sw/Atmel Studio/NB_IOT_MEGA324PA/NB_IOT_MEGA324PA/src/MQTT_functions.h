/*
 * MQTT_functions.h
 *
 * Created: 28/06/2017 17:12:09
 *  Author: jan.rune.herheim
 */ 


#ifndef MQTT_FUNCTIONS_H_
#define MQTT_FUNCTIONS_H_

#include "MQTTPacket.h"

int mqtt_packet(char *payload, char *package, int buflen, char *sub_topic);


#endif /* MQTT_FUNCTIONS_H_ */