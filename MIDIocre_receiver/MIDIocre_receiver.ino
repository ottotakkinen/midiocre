#include <esp_now.h>
#include <WiFi.h>

#define RXp2 16
#define TXp2 17

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int16_t x;
  int16_t y;
  int16_t z;
  int16_t pressure1;  
} struct_message;

struct_message sensorData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&sensorData, incomingData, sizeof(sensorData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Pressure 1: ");
  Serial.println(sensorData.pressure1);
  //Serial2.write((char *)incomingData);
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  Serial2.begin(28800, SERIAL_8N1, RXp2, TXp2);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  send(&sensorData);
  delay(100);
}

void send (const struct_message* data)
{
  Serial2.write((const char*)data, sizeof(struct_message));  // 8 bytes.
}
