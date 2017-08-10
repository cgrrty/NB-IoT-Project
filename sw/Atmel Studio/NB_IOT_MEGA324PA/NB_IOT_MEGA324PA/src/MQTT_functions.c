/*
 * MQTT_functions.c
 *
 * Created: 28/06/2017 15:22:07
 *  Author: jan.rune.herheim
 */
#include "string.h"
#include "MQTT_functions.h" 

/////////////MQTT////////////////////
int mqtt_packet(char *payload, char *package, int buflen, char *sub_topic)
{
	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
	
	//volatile int buflen = sizeof(&package);
	char buf[256];
	//buflen = sizeof(buf);
	MQTTString topicString = MQTTString_initializer;
	
	int payloadlen = strlen(payload);
	int len = 0;
	
	data.clientID.cstring = "LE";
	//data.clientID.cstring = MQTT_TOPIC; //FIX SO THIS ONE WORKS, COULD BE AN ADDITIONAL INPUT.
	data.keepAliveInterval = 20;
	data.cleansession = 1;
	data.username.cstring = "";
	data.password.cstring = "";
	data.MQTTVersion = 4;

	len = MQTTSerialize_connect((unsigned char *)package, buflen, &data);
	
	//topicString.cstring = "LE/0"; //MAKE A GENERAL CONFIGUREATION OF THIS PARAMETER
	char main_sub_topic[6] = "LE/";
	strcat(main_sub_topic, sub_topic);
	topicString.cstring = main_sub_topic;
	
	
	len += MQTTSerialize_publish((unsigned char *)(package + len), buflen - len, 0, 0, 0, 0, topicString, (unsigned char *)payload, payloadlen);
		
	len += MQTTSerialize_disconnect((unsigned char *)(package + len), buflen - len);
	
	exit:
	
	//return 0;
	return len;
}

///////////////////////////////////////////////////


/////////////MQTT////////////////////
/*
int mqtt_packet(char *payload, char *package)
{
	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
	
	int buflen = sizeof(package);
	MQTTString topicString = MQTTString_initializer;
	
	int payloadlen = strlen(payload);
	int len = 0;
	
	data.clientID.cstring = "M95";
	data.keepAliveInterval = 20;
	data.cleansession = 1;
	data.username.cstring = "";
	data.password.cstring = "";
	data.MQTTVersion = 4;

	len = MQTTSerialize_connect((unsigned char *)package, buflen, &data);

	topicString.cstring = "home/garden/fountain";
	len += MQTTSerialize_publish((unsigned char *)(package + len), buflen - len, 0, 0, 0, 0, topicString, (unsigned char *)payload, payloadlen);
	
	len += MQTTSerialize_disconnect((unsigned char *)(package + len), buflen - len);

	exit:
	
	return 0;
}
*/
///////////////////////////////////////////////////
