
#ifndef __CC3200R1M1RGC__
// No need to include SPI.h for CC3200
#include <SPI.h>
#endif
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include"AirQuality.h"

#define HALL_SENSOR_PIN 19
#define TILT_SWITCH_PIN 18
#define AIR_QUALITY_PIN 24

/* AP Mode const and variables */
const char ssid[] = "VC_AP_Mode";
const char wifipw[] = "veloconnecte";
const uint32_t AP_MODE_REFRESH_FREQUENCY = 200;

unsigned int num_clients = 0;
uint32_t ap_mode_timeold; // date of the last refresh
WiFiServer server(80);

/* Captors const and variables */
const uint32_t MEASURE_FREQUENCY = 5000; // 5s = 5000ms : time period between two measurments.
const uint32_t AIR_MEASURE_FREQUENCY = 1000; // 5s = 5000ms : time period between two measurments.

volatile uint8_t rpmcount;//(rpmcount - 1) is the number of revolutions 
volatile uint8_t tiltcount;//(tiltcountount - 1) is the number of tilts 
uint16_t rpm;//revolutions per minute
uint16_t tilt;//tilts over measure period
uint32_t timeold; // date of the last measure
uint32_t airquality_timeold; // date of the last air quality measure
AirQuality airqualitysensor;
uint16_t mean_airquality = 0;
uint32_t airquality_buffer[5]; 

void setup()
{
  Serial.begin(9600);
  ap_mode_setup();
  sensors_setup();
}
 
void loop()
{
  ap_mode_handler();
  webserver_handler();
  measurments_handler();
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
  printWifiStatus();
  
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

void webserver_handler() {
  // listen for incoming clients
  WiFiClient client = server.available();
  if (client) {
    Serial.println("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println("Refresh: 5");  // refresh the page automatically every 5 sec
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
          // output the value of each analog input pin
          /*
          for (int analogChannel = 0; analogChannel < 6; analogChannel++) {
            int sensorReading = analogChannel; //analogRead(analogChannel);
            client.print("analog input ");
            client.print(analogChannel);
            client.print(" is ");
            client.print(sensorReading);
            client.println("<br />");
          }
          */
          client.println("<h1> Velo connecte : </h1>");     
          client.println("RPM : ");
          client.println(rpm);   
          client.println("( ");
          client.println(rpm / 12);
          client.println(" tours)");
          client.println("<br/>Tilts : ");
          client.println(tilt);     
          client.println("<br/>Air quality : ");
          client.println(mean_airquality);     
          client.println("</html>");
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        }
        else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);

    // close the connection:
    client.stop();
    Serial.println("client disonnected");
  }
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("Network Name: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

/* --- Sensor handlers --- */
void sensors_setup(){

  pinMode(HALL_SENSOR_PIN, INPUT);
  pinMode(TILT_SWITCH_PIN, INPUT);
  pinMode(AIR_QUALITY_PIN, INPUT);
  
  //attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), rpm_fun, RISING); // Supposedly better, but digitalPinToInterrupt is not recognized.
  attachInterrupt(HALL_SENSOR_PIN, rpm_fun, RISING); // Interrupt triggers on rising edge; 
                                                   //when the sensor turns off(the magnet leaves).
  attachInterrupt(TILT_SWITCH_PIN, tilt_fun, RISING); // Interrupt triggers on rising edge; 
                                                   //when the sensor turns off(the magnet leaves).
  rpm = 0;
  tilt = 0;

  airquality_buffer_init();
  
                                                   
}

void measurments_handler(){
  
  if (millis() - airquality_timeold > AIR_MEASURE_FREQUENCY) 
  {   
    airquality_buffer_update();
    airquality_timeold = millis();
  }
  
  if (millis() - timeold > MEASURE_FREQUENCY) 
  {   
    
    hall_handler();
    tilt_handler();
    mean_airquality_compute();
    timeold = millis();
    
  }
}

void hall_handler(){
    //Update RPM every 5s, increase this for better RPM resolution,
    //decrease for faster update
   if(rpmcount > 0){
      rpm = 60000/MEASURE_FREQUENCY*(rpmcount-1);//1 min = 60000ms
   }else{
      rpm = 0;
   }   
    rpmcount = 0;
}
void tilt_handler(){
   

    if(tiltcount > 0){
       tilt = tiltcount-1;//1 min = 60000ms
    }else{
       tilt = 0;
    }
    tiltcount = 0;
    
}
void rpm_fun()
{
  rpmcount++;
}
void tilt_fun()
{
  tiltcount++;
}

/* ------ Air quality buffer handler --------- */
void airquality_buffer_init(){
  airquality_buffer[0] = 0;
  airquality_buffer[1] = 0;
  airquality_buffer[2] = 0;
  airquality_buffer[3] = 0;
  airquality_buffer[4] = 0;
}

void airquality_buffer_update(){
  uint32_t aq_measure = analogRead(AIR_QUALITY_PIN);
  airquality_buffer[0] = airquality_buffer[1];
  airquality_buffer[1] = airquality_buffer[2];
  airquality_buffer[2] = airquality_buffer[3];
  airquality_buffer[3] = airquality_buffer[4];
  airquality_buffer[4] = aq_measure;
}

void mean_airquality_compute(){
  mean_airquality = (airquality_buffer[0] + airquality_buffer[1] + airquality_buffer[2] + airquality_buffer[3] + airquality_buffer[4]) / 5;
}

/* ----------------------- */
