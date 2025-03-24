#include <WiFi.h>
#include <esp_now.h>

#define RIGHT_X_PIN 34  // gpio 34 (adc1_ch6)
#define RIGHT_Y_PIN 32  // ?
#define LEFT_X_PIN 0    //
#define LEFT_Y_PIN 35   // gpio 35 (adc1_ch7)

#define LEFT_BUTTON 25
#define RIGHT_BUTTON 19
#define FLOAT_BUTTON 12

#define PWR_LED 22
#define COM_LED 23

#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

class SlewRateLimiter {
  float lastValue;
  float allowedDeltaPositive;
  float allowedDeltaNegative;

public:
  void init(float allowedDeltaPositive, float allowedDeltaNegative) {
    this->allowedDeltaPositive = allowedDeltaPositive;
    this->allowedDeltaNegative = allowedDeltaNegative;
    lastValue = NULL;
  }
  float calculate(float nextValue) {
    float delta = nextValue - lastValue;
    if (lastValue == NULL) {
      lastValue = nextValue;
    } else if (delta > allowedDeltaPositive) {
      lastValue += allowedDeltaPositive;
    } else if (delta < allowedDeltaNegative) {
      lastValue -= allowedDeltaNegative;
    } else {
      lastValue = nextValue;
    }
    return lastValue;
  }
};


enum DriveType {
  ARCADE,
  TANK,
};
enum ArmPosition {
  UP,
  DOWN,
  REST,
};
uint8_t broadcastAddress[] = { 0xcc, 0xdb, 0xa7, 0x98, 0xb6, 0xb8 };  // cc:db:a7:98:b6:b8 mine


typedef struct struct_message {
  DriveType mode;
  ArmPosition armPos;
  float leftX;
  float leftY;
  float rightX;
  float rightY;
  int horn;
} struct_message;

SlewRateLimiter speedlim;
SlewRateLimiter dirlim;
struct_message sendData;
bool comm_est = false;
DriveType mode = ARCADE;
ArmPosition armPos = REST;
esp_now_peer_info_t peerInfo;
bool lastModeState = false;
bool lastArmButtonState = false;
ArmPosition lastArmPos = REST;
int armResetTimeout = 0;



// callback when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    comm_est = false;
  } else {
    comm_est = true;
  }
}


void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(onDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // set pin types
  pinMode(LEFT_BUTTON, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON, INPUT_PULLUP);
  pinMode(FLOAT_BUTTON, INPUT_PULLUP);

  pinMode(PWR_LED, OUTPUT);
  pinMode(COM_LED, OUTPUT);

  pinMode(RIGHT_X_PIN, INPUT);
  pinMode(RIGHT_Y_PIN, INPUT);
  pinMode(LEFT_Y_PIN, INPUT);

  // set pin attenuation so we don't blow the adc
  analogSetPinAttenuation(RIGHT_X_PIN, ADC_ATTENDB_MAX);
  analogSetPinAttenuation(LEFT_Y_PIN, ADC_ATTENDB_MAX);
  analogSetPinAttenuation(RIGHT_Y_PIN, ADC_ATTENDB_MAX);


  // SlewRateLimiters
  speedlim.init(0.05, 0.05);
  dirlim.init(0.05, 0.05);
}

void loop() {
  // Set values to send
  bool modeButtonState = !digitalRead(LEFT_BUTTON);
  bool hornButtonState = !digitalRead(RIGHT_BUTTON);
  bool armButtonState = !digitalRead(FLOAT_BUTTON);
  float leftY = (analogRead(LEFT_Y_PIN) / 4095.0) * 2 - 1;    // tank left; ARCADE speed
  float leftX = 0;                                            // unused -- perhaps servo control?
  float rightX = (analogRead(RIGHT_X_PIN) / 4095.0) * 2 - 1;  // ARCADE direction
  float rightY = (analogRead(RIGHT_Y_PIN) / 4095.0) * 2 - 1;  // tank right

  if (modeButtonState != lastModeState && modeButtonState == HIGH) {
    mode = mode == TANK ? ARCADE : TANK;  // toggle tank and arcade
  }
  lastModeState = modeButtonState;

  if (armButtonState != lastArmButtonState && armButtonState == HIGH) {
    armPos = lastArmPos == UP ? DOWN : UP;
    lastArmPos = armPos;

    armResetTimeout = 0;
  }
  if (armPos != REST) {
    armResetTimeout++;
  }

  if (armResetTimeout > 20) {
    armPos = REST;
    armResetTimeout = 0;
  }

  lastArmButtonState = armButtonState;

  // deadzones
  float deadzone = 0.2;
  if (abs(leftY) < deadzone) leftY = 0;
  if (abs(leftX) < deadzone) leftX = 0;
  if (abs(rightY) < deadzone) rightY = 0;
  if (abs(rightX) < deadzone) rightX = 0;

  sendData.mode = mode;
  sendData.armPos = armPos;
  sendData.leftY = leftY * leftY * -sgn(leftY);
  sendData.leftX = leftX * leftX * sgn(leftX);
  sendData.rightX = rightX * rightX * sgn(rightX);
  sendData.rightY = rightY * rightY * -sgn(rightY);
  sendData.horn = hornButtonState;

  Serial.print(",ModeState:");
  Serial.print(sendData.mode);
  Serial.print(",HornState:");
  Serial.print(hornButtonState);
  Serial.print(",ArmState:");
  Serial.print(sendData.armPos);
  Serial.print(",ArmTimout:");
  Serial.print(armResetTimeout);
  Serial.print(",LX:");
  Serial.print(sendData.leftX);
  Serial.print(",LY:");
  Serial.print(sendData.leftY);
  Serial.print(",RX:");
  Serial.print(sendData.rightX);
  Serial.print(",RY:");
  Serial.println(sendData.rightY);

  if (comm_est) {
    digitalWrite(COM_LED, HIGH);
  } else {
    digitalWrite(COM_LED, LOW);
  }
  if (!sendData.mode) {
    digitalWrite(PWR_LED, HIGH);
  } else {
    digitalWrite(PWR_LED, LOW);
  }

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&sendData, sizeof(sendData));

  // if (result == ESP_OK) {
  //   Serial.println("send ok");
  // }
  // else {
  //   Serial.println("send fail");
  // }
  delay(25);
}