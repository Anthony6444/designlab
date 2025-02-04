#include <esp_now.h>
#include <WiFi.h>

// Structure example to receive data
// Must match the sender structure

#define LED_PIN 2

#define M1A 25
#define M1B 26
#define M2A 32
#define M2B 33

#define M1PWM 18
#define M2PWM 19


class Motor {
  private:
    int pinA;
    int pinB;
    int pinPWM;
    bool reversed;
  public:
    void init(int pinA, int pinB, int pinPWM, bool reversed) {
      this->pinA = pinA;
      this->pinB = pinB;
      this->pinPWM = pinPWM;
      this->reversed = reversed;

      pinMode(pinA, OUTPUT);
      pinMode(pinB, OUTPUT);
      pinMode(pinPWM, OUTPUT);
      ledcAttachChannel(pinPWM, 30000, 8, 0);
    }
    void forward(float speed) {
      int trueSpeed = 255 * speed;
      Serial.println(trueSpeed);
      ledcWrite(pinPWM, trueSpeed);
      digitalWrite(pinA, HIGH);
      digitalWrite(pinB, LOW);
    }
    void stop() {
      // analogWrite(pinPWM, LOW);
      digitalWrite(pinA, LOW);
      digitalWrite(pinB, LOW);
    }
    void backward(float speed) {
      int trueSpeed = 255 * speed;
      ledcWrite(pinPWM, trueSpeed);
      digitalWrite(pinA, LOW);
      digitalWrite(pinB, HIGH);
    }

    void drive(float speed) {
      if (speed > 0.1) {
        forward(speed);
      } else if (speed < -0.1) {
        backward(speed);
      } else {
        stop();
      }
    }
};

enum DriveType {
  ARCADE,
  TANK,
};

typedef struct struct_message {
  DriveType mode;
  float leftX;
  float leftY;
  float rightX;
  float rightY;
  int message;
} struct_message;

// Create a struct_message called recvData
struct_message recvData;
bool newData = false;

// callback function that will be executed when data is received
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&recvData, incomingData, sizeof(recvData));
  newData = true;
}

Motor left;
Motor right;

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
  esp_now_register_recv_cb(esp_now_recv_cb_t(onDataRecv));

  pinMode(LED_PIN, OUTPUT);
  left.init(M1A, M1B, M1PWM, false);
  right.init(M2A, M2B, M2PWM, true);

}


void loop() {

  // left.drive(1);
  // delay(1000);
  // left.stop();
  // delay(1000);
  // left.drive(-1);
  // delay(1000);
  // left.stop();
  // delay(1000);

  if (newData) {
    // do things

    Serial.print(recvData.leftY);
    Serial.print(", ");
    Serial.println(recvData.rightY);
    
    float xSpeed, zRot, leftSpeed, rightSpeed, greaterInput, lesserInput, saturatedInput;
    switch (recvData.mode) {
      case ARCADE:
        // handle arcade driving

        xSpeed = recvData.leftY;
        zRot   = recvData.rightX;

        leftSpeed  = xSpeed - zRot;
        rightSpeed = xSpeed + zRot;

        greaterInput = max(abs(xSpeed), abs(zRot));
        lesserInput  = min(abs(xSpeed), abs(zRot));
        
        if (greaterInput == 0.0) {
          left.stop();
          right.stop();
        }

        saturatedInput = (greaterInput + lesserInput) / greaterInput;
        leftSpeed /= saturatedInput;
        rightSpeed /= saturatedInput;

        left.drive(leftSpeed);
        right.drive(rightSpeed);

        break;
      case TANK: 
        // handle tank drive
        left.drive(recvData.leftY);
        right.drive(recvData.rightY);
    }

    newData = false;
  }
  delay(100);
}