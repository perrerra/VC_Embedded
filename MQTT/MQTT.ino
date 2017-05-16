
#ifndef __CC3200R1M1RGC__
// No need to include SPI.h for CC3200
#include <SPI.h>
#endif
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <Wire.h>
#include <BMA222.h>
#include <WifiIPStack.h>
#include <Countdown.h>
#include <MQTTClient.h>

/* AP Mode const and variables */
const char ssid[] = "VC_AP_Mode";
const char wifipw[] = "veloconnecte";
const uint32_t AP_MODE_REFRESH_FREQUENCY = 200;

unsigned int num_clients = 0;
uint32_t ap_mode_timeold; // date of the last refresh
WiFiServer server(80);

/* MQTT const and variables */

const char* MQTT_TOPIC = "velo";
const int MQTT_BROKER_PORT = 1883;


WifiIPStack ipstack;
MQTT::Client<WifiIPStack, Countdown> client = MQTT::Client<WifiIPStack, Countdown>(ipstack);
char mqtt_printbuf[100];
int mqtt_arrivedcount = 0;

void setup()
{
  Serial.begin(9600);
  ap_mode_setup();

}
 
void loop()
{
  ap_mode_handler();
  mqtt_handler();
}

/* ---  AP mode ---- */

void ap_mode_setup(){
  Serial.print("Setting up Access Point named: ");
  Serial.println(ssid);
  Serial.print("AP uses WPA and password is: ");
  Serial.println(wifipw);
  
  WiFi.beginNetwork((char *)ssid, (char *)wifipw);
  while (WiFi.localIP() == INADDR_NONE) {
    // print dots while we wait for the AP config to complete
    Serial.print('.');
    delay(300);
  }
  Serial.println();
  Serial.println("AP active.");
  // you're connected now, so print out the status  

  
  Serial.println("Starting webserver on port 80");
  server.begin();                           // start the web server on port 80
  Serial.println("Webserver started!");
  
}

void ap_mode_handler()
{
  unsigned int a, i;
  if(millis() - ap_mode_timeold > AP_MODE_REFRESH_FREQUENCY){
         
    a = WiFi.getTotalDevices();
    
    // Did a client connect/disconnect since the last time we checked?
    if (a != num_clients) {
      if (a > num_clients) {  // Client connect
        digitalWrite(RED_LED, !digitalRead(RED_LED));
        Serial.println("Client connected! All clients:");
        for (i = 0; i < a; i++) {
          Serial.print("Client #");
          Serial.print(i);
          Serial.print(" at IP address = ");
          Serial.print(WiFi.deviceIpAddress(i));
          Serial.print(", MAC = ");
          Serial.println(WiFi.deviceMacAddress(i));
        }
      } else {  // Client disconnect
        digitalWrite(RED_LED, !digitalRead(RED_LED));
        Serial.println("Client disconnected.");
      }    
      num_clients = a;
    }
    ap_mode_timeold = millis();
  }
}

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


void mqtt_handler()
{
  if (!client.isConnected())
    mqtt_connect();
  
  MQTT::Message message;
  
  mqtt_arrivedcount = 0;

  // Send and receive QoS 0 message
  char buf[100];
  sprintf(buf, "Hello World! QoS 0 message");
  Serial.println(buf);
  message.qos = MQTT::QOS0;
  message.retained = false;
  message.dup = false;
  message.payload = (void*)buf;
  message.payloadlen = strlen(buf)+1;
  int rc = client.publish(MQTT_TOPIC, message);
  
  while (mqtt_arrivedcount == 0)
    client.yield(1000);
        
  // Send and receive QoS 1 message
  sprintf(buf, "Hello World!  QoS 1 message");
  Serial.println(buf);
  message.qos = MQTT::QOS1;
  message.payloadlen = strlen(buf)+1;
  rc = client.publish(MQTT_TOPIC, message);
  while (mqtt_arrivedcount == 1)
    client.yield(1000);
        
  // Send and receive QoS 2 message
  sprintf(buf, "Hello World!  QoS 2 message");
  Serial.println(buf);
  message.qos = MQTT::QOS2;
  message.payloadlen = strlen(buf)+1;
  rc = client.publish(MQTT_TOPIC, message);
  while (mqtt_arrivedcount == 2)
    client.yield(1000);
  
  delay(2000);
  
}


void mqtt_messageArrived(MQTT::MessageData& md)
{
  MQTT::Message &message = md.message;
  
  sprintf(mqtt_printbuf, "Message %d arrived: qos %d, retained %d, dup %d, packetid %d\n", 
    ++mqtt_arrivedcount, message.qos, message.retained, message.dup, message.id);
  Serial.print(mqtt_printbuf);
  sprintf(mqtt_printbuf, "Payload %s\n", (char*)message.payload);
  Serial.print(mqtt_printbuf);
}

