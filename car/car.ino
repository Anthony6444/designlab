#include <esp_now.h>
#include <WiFi.h>

// Structure example to receive data
// Must match the sender structure

int ledPin = 2;
int motorForwardPin = 12;
int motorReversePin = 14;

typedef struct struct_message {
  bool channel0;
  double speed;
  double direction;
} struct_message;

// Create a struct_message called recvData
struct_message recvData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&recvData, incomingData, sizeof(recvData));
  Serial.print("ch0:");
  Serial.print(recvData.channel0);
  Serial.print(",spd:");
  Serial.print(recvData.speed);
  Serial.print(",dir:");
  Serial.println(recvData.direction);
  digitalWrite(ledPin, (bool)recvData.channel0);

  if (recvData.speed > 0.2) {
    digitalWrite(motorForwardPin, 1);
    digitalWrite(motorReversePin, 0);
  } else if (recvData.speed < -0.2) {
    digitalWrite(motorReversePin, 1);
    digitalWrite(motorForwardPin, 0);
  } else {
    digitalWrite(motorForwardPin, 0);
    digitalWrite(motorReversePin, 0);
  }

}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  pinMode(ledPin, OUTPUT);
}

void loop() {
  digitalWrite(ledPin, 1);
  delay(200);
  digitalWrite(ledPin, 0);
  delay(400);
}