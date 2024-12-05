#include <WiFi.h>
#include <esp_now.h>

#define BUTTON_PIN 12
#define POT1_PIN 35 // gpio 35 (adc1_ch7)
#define POT2_PIN 34 // gpio 34 (adc1_ch6)

uint8_t broadcastAddress[] = {0xcc, 0xdb, 0xa7, 0x99, 0x60, 0x40};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  bool channel0;
  double speed;
  double direction;
} struct_message;

struct_message sendData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("Last packet status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "delivered ok" : "failed delivery");
}
 
void setup() {
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of transmitted packet
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

  pinMode(BUTTON_PIN, INPUT);
}
 
void loop() {
  // Set values to send
  sendData.channel0 =   digitalRead(BUTTON_PIN);
  sendData.speed =      analogRead(POT1_PIN) / 4095.0;
  sendData.direction = (analogRead(POT2_PIN) / 4095.0)*2 - 1;
  Serial.print("Channel_0:");
  Serial.print(sendData.channel0);

  Serial.print(",Speed:");
  Serial.print(sendData.speed); 

  Serial.print(",Direction:");
  Serial.println(sendData.direction);
  
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sendData, sizeof(sendData));
   
  // if (result == ESP_OK) {
  //   Serial.println("send ok");
  // }
  // else {
  //   Serial.println("send fail");
  // }
  delay(25);
}