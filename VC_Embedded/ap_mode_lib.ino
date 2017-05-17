
#ifndef __CC3200R1M1RGC__
// No need to include SPI.h for CC3200
#include <SPI.h>
#endif
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <Wire.h>


/* AP Mode const and variables */
const char ssid[] = "VC_AP_Mode";
const char wifipw[] = "veloconnecte";
const uint32_t AP_MODE_REFRESH_FREQUENCY = 200;

unsigned int num_clients = 0;
uint32_t ap_mode_timeold; // date of the last refresh
WiFiServer server(80);

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


