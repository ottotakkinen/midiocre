#include <SoftwareSerial.h>
SoftwareSerial softSerial(2, 3);

typedef struct struct_message {
  int16_t x;
  int16_t y;
  int16_t z;
  int16_t pressure1;  
} struct_message;

struct_message sensorData;

/*struct SensorData {
  int16_t x;
  int16_t y;
  int16_t z;
  int16_t pressure1;  
};
struct SensorData data= {0,0,0,420}; */


void setup()  
{
  softSerial.begin(28800);
  Serial.begin(115200);
}

void loop()  
{ 
  receive(&sensorData);
  
  
  /*Serial.print("X: ");    
  Serial.println(data.x);
  Serial.print("Y: ");
  Serial.println(data.y);
  Serial.print("Z: ");
  Serial.println(data.z);*/
  Serial.print("pressure1: ");
  Serial.println(sensorData.pressure1);

}

bool receive(const struct_message* table)
{
  return (softSerial.readBytes((char*)table, sizeof(struct_message)) == sizeof(struct_message));
}

/*bool receive(const struct_message* table)
{
  return (softSerial.readBytes((char*)table, sizeof(struct_message)) == sizeof(struct_message));
}*/
