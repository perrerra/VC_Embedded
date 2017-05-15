
#define hallSensorPin 18

volatile uint8_t rpmcount;//(rpmcount - 1) is the number of revolutions
 
uint16_t rpm;//revolutions per minute
uint32_t timeold; // date of the last measure
const uint32_t ROTATING_TIME = 5000; // 5s = 5000ms : time period between two measurments.



void setup()
{
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(hallSensorPin), rpm_fun, RISING); // Interrupt triggers on rising edge; 
                    //when the sensor turns off(the magnet leaves).
}
 
void loop()
{
  if (millis() - timeold > ROTATING_TIME) 
  { 
  
    //Update RPM every 5s, increase this for better RPM resolution,
    //decrease for faster update
    rpm = 60000/ROTATING_TIME*(rpmcount-1);//1 min = 60000ms
    rpmcount = 0;
    timeold = millis;
    Serial.println(rpm, DEC);
  }
}
 
void rpm_fun()
{
  rpmcount++;
}
