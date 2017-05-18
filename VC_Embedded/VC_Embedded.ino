
#ifndef __CC3200R1M1RGC__
// No need to include SPI.h for CC3200
#include <SPI.h>
#endif

#include <Wire.h>
#include <BMA222.h>

#include"AirQuality.h"

#define HALL_SENSOR_PIN 19
#define TILT_SWITCH_PIN 18
#define AIR_QUALITY_PIN 24

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
BMA222 imu_sensor;
int8_t xaxis;
int8_t yaxis;
int8_t zaxis;
float slope_value;

void setup()
{
  Serial.begin(9600);
  ap_mode_setup();
  sensors_setup();
  mpu6050_setup();
}
 
void loop()
{
  ap_mode_handler();
  measurments_handler();
  mpu6050_computing();
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
  xaxis = 0;
  yaxis = 0;
  zaxis = 0;
  slope_value = 0;
  
  // Accelerometer
  imu_sensor.begin();
  uint8_t chipID = imu_sensor.chipID();                                                 
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
    accelerometer_read();
    timeold = millis();  
    mqtt_publish_values();  
    slope_value = getSlope();
    Serial.println(slope_value);
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

void mqtt_publish_values(){
  mqtt_send("vc_rpm", rpm);
  mqtt_send("air_quality", mean_airquality);
  mqtt_send("tilt", tilt);  
  mqtt_send("slope", slope_value);
  
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

void accelerometer_read()
{
  xaxis = imu_sensor.readXData();
  yaxis = imu_sensor.readYData();
  zaxis = imu_sensor.readZData(); 
}

/* ----------------------- */

