#include <esp_now.h>
#include <WiFi.h>

// Structure example to receive data
// Must match the sender structure

int ledPin = 2;
uint8_t ledSpeed[7] = { 25, 26, 27, 32, 33, 12, 13 };
uint8_t ledDir[7] = { 16, 17, 18, 19, 21, 22, 23 };


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
  int binSpeed = constrain((int)(recvData.speed * 7), 0, 7);
  int binDir = constrain((int)(((recvData.direction + 1.0) / 2.0) * 7), 0, 6);
  for (int i = 0; i < 7; i++) digitalWrite(ledSpeed[i], i < binSpeed ? HIGH : LOW);
  for (int i = 0; i < 7; i++) digitalWrite(ledDir[i], i == binDir ? HIGH : LOW);
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
  for (int i = 0; i < 7; i++) {
    pinMode(ledSpeed[i], OUTPUT);
    pinMode(ledDir[i], OUTPUT);
  }
  for (int i = 0; i < 7; i++) {
    digitalWrite(ledSpeed[i], HIGH);
    digitalWrite(ledDir[i], HIGH);
    delay(50);
    digitalWrite(ledDir[i], LOW);
  }
  delay(500);
  for (int i = 6; i >= 0; i--) {
    digitalWrite(ledSpeed[i], LOW);
    digitalWrite(ledDir[i], HIGH);
    delay(50);
    digitalWrite(ledDir[i], LOW);
  }
}

void loop() {
}