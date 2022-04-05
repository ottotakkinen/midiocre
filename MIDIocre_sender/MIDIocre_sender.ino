#include <esp_now.h>
#include <WiFi.h>

#define FORCE_SENSOR_PIN 32
#define FORCE_SENSOR_PIN_2 33

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);

uint8_t broadcastAddress[] = {0x78, 0x21, 0x84, 0x8C, 0xC2, 0xBC};

//Must match the receiver structure
typedef struct struct_message {
  int16_t x;
  int16_t y;
  int16_t z;
  int16_t pressure1;  
  int16_t pressure2;
} struct_message;

struct_message sensorData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup(){
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  /* Initialise the sensor */
  if(!bno.begin()) {
  /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}
 
void loop(){
  
  sensorData.pressure1 = analogRead(FORCE_SENSOR_PIN);
  sensorData.pressure2 = analogRead(FORCE_SENSOR_PIN_2);
  
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sensorData, sizeof(sensorData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  sensors_event_t event;
  bno.getEvent(&event);
  
  /* Display the floating point data */
  
  sensorData.z = (event.orientation.z);
  sensorData.y = (event.orientation.y);
  
  if (sensorData.z >= 90)
    sensorData.z = 180 - sensorData.z;
  if (sensorData.z <= -90)
    sensorData.z = -180 - sensorData.z;
    
  if (sensorData.y >= 90)
    sensorData.y = 180 - sensorData.y;
  if (sensorData.y <= -90)
    sensorData.y = -180 - sensorData.y; 
  
  Serial.print("Y: ");
  Serial.println(sensorData.y);
  
  Serial.print("Z: ");
  Serial.println(sensorData.z);
  
  Serial.print("pressure1: ");
  Serial.println(sensorData.pressure1);
  
  Serial.print("pressure2: ");
  Serial.println(sensorData.pressure2);
  
  delay(100);
}
