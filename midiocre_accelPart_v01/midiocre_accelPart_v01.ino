#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup(void)
{

Serial.begin(9600);
Serial.println("Orientation Sensor Test"); Serial.println("");
/* Initialise the sensor */
if(!bno.begin())
{
/* There was a problem detecting the BNO055 ... check your connections */
Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
while(1);
}
delay(1000);
bno.setExtCrystalUse(true);
}





void loop(void)
{
int rotZ = 0;
int rotY = 0;

sensors_event_t event;
bno.getEvent(&event);

/* Display the floating point data */

rotZ = (event.orientation.z);
rotY = (event.orientation.y);

if (rotZ >= 90)
  rotZ = 180 - rotZ;
if (rotZ <= -90)
  rotZ = -180 - rotZ;
  
if (rotY >= 90)
  rotY = 180 - rotY;
if (rotY <= -90)
  rotY = -180 - rotY; 

Serial.println(rotY);
Serial.println(rotZ);

delay(100);
}
