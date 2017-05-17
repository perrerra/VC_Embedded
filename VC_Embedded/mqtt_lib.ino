
#ifndef __CC3200R1M1RGC__
// No need to include SPI.h for CC3200
#include <SPI.h>
#endif

#include <WifiIPStack.h>
#include <Countdown.h>
#include <MQTTClient.h>


/* MQTT const and variables */
const char* MQTT_TOPIC = "velo";
const int MQTT_BROKER_PORT = 1883;

WifiIPStack ipstack;
MQTT::Client<WifiIPStack, Countdown> client = MQTT::Client<WifiIPStack, Countdown>(ipstack);
char mqtt_printbuf[100];
int mqtt_arrivedcount = 0;

/* MQTT */
void mqtt_connect()
{
  char hostname[] = "192.168.1.3";

  sprintf(mqtt_printbuf, "Connecting to %s:%d\n", hostname, MQTT_BROKER_PORT);
  Serial.print(mqtt_printbuf);
  int rc = ipstack.connect(hostname, MQTT_BROKER_PORT);
  if (rc != 1)
  {
    sprintf(mqtt_printbuf, "rc from TCP connect is %d\n", rc);
    Serial.print(mqtt_printbuf);
  }
 
  Serial.println("MQTT connecting");
  MQTTPacket_connectData data = MQTTPacket_connectData_initializer;       
  data.MQTTVersion = 3;
  data.clientID.cstring = (char*)"veloconnecte";
  rc = client.connect(data);
  if (rc != 0)
  {
    sprintf(mqtt_printbuf, "rc from MQTT connect is %d\n", rc);
    Serial.print(mqtt_printbuf);
  }
  Serial.println("MQTT connected");
 /*
  rc = client.subscribe(MQTT_TOPIC, MQTT::QOS2, mqtt_messageArrived);   
  if (rc != 0)
  {
    sprintf(mqtt_printbuf, "rc from MQTT subscribe is %d\n", rc);
    Serial.print(mqtt_printbuf);
  }
  */
  Serial.println("MQTT subscribed");
 
}
void mqtt_send(char topic[], int value)
{
  char buf[100];
  sprintf(buf, "%d", value);
  mqtt_send(topic, buf);
}

void mqtt_send(char topic[], char message_content[])
{
  if (!client.isConnected())
    mqtt_connect();
  
  MQTT::Message message;
  
  mqtt_arrivedcount = 0;

  // Send and receive QoS 0 message
  char buf[100];
  sprintf(buf, message_content);
  Serial.println(buf);
  message.qos = MQTT::QOS0;
  message.retained = false;
  message.dup = false;
  message.payload = (void*)buf;
  message.payloadlen = strlen(buf)+1;
  int rc = client.publish(topic, message);  
}

/*
void mqtt_messageArrived(MQTT::MessageData& md)
{
  MQTT::Message &message = md.message;
  
  sprintf(mqtt_printbuf, "Message %d arrived: qos %d, retained %d, dup %d, packetid %d\n", 
    ++mqtt_arrivedcount, message.qos, message.retained, message.dup, message.id);
  Serial.print(mqtt_printbuf);
  sprintf(mqtt_printbuf, "Payload %s\n", (char*)message.payload);
  Serial.print(mqtt_printbuf);
}
*/
