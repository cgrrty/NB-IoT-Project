/*
 * user_network_params.h
 *
 * Created: 11/05/2017 08:11:32
 *  Author: jan.rune.herheim
 */ 


#ifndef USER_NETWORK_PARAMS_H_
#define USER_NETWORK_PARAMS_H_

#define SMS_RECEIVER "+4798480520"
//#define APN "apn1.lillebakk.com"
#define APN "public"
#define USERNAME ""
#define PASSWORD ""

#define IP_MODE "TCP"
//#define SERVER_ADDR "10.18.0.39" //MQTT server (broker) on remote desktop
#define SERVER_ADDR "85.119.83.194" //test.mosquitto.org MQTT test server
#define SERVER_PORT "1883" //MQTT server port on remote desktop

//#define  LOCAL_IP "10.10.0.13\r\n" //CHANGE WITH \r\n IN THE COMMAND SECTION.
#define  LOCAL_IP "185.67.18.74\r\n" //CHANGE WITH \r\n IN THE COMMAND SECTION.

#define MQTT_TOPIC "LE/"
char mqtt_sub_topic[3] = "";
//char mqtt_sub_topic = "AVG";


#endif /* USER_NETWORK_PARAMS_H_ */