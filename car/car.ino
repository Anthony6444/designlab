#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

// Structure example to receive data
// Must match the sender structure

#define LED_PIN 2
#define SERVO_PIN 13
#define BUZZER_PIN 18
#define M1A 26  // motor 1 pin A
#define M1B 25  // and so on
#define M2A 32
#define M2B 33

#define LED_R 14  // R LED pin
#define LED_G 12  // G LED pin
#define LED_B 27  // B LED pin

// 12 bit precision for timer
#define LEDC_TIMER_12_BIT 12
// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ 5000

void ledcAnalogWrite(uint8_t pin, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 4095 from 2 ^ 12 - 1
  uint32_t duty = (4095 / valueMax) * min(value, valueMax);

  // write duty to LEDC
  ledcWrite(pin, duty);
}

void showColor(float r, float g, float b, float a = 255.0) {
  ledcAnalogWrite(LED_R, r * a);
  ledcAnalogWrite(LED_G, g * a);
  ledcAnalogWrite(LED_B, b * a);
}

class Motor {
private:
  int pinA;
  int pinB;
  bool reversed;
public:
  void init(int pinA, int pinB, bool reversed) {
    this->pinA = pinA;
    this->pinB = pinB;
    this->reversed = reversed;

    // Setup timer and attach timer to a led pin
    ledcAttach(pinA, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
    ledcAttach(pinB, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  }
  void forward(float speed) {
    int trueSpeed = 255 * speed;
    ledcAnalogWrite(pinA, trueSpeed);
    ledcAnalogWrite(pinB, 0);
  }
  void stop() {
    ledcAnalogWrite(pinA, 0);
    ledcAnalogWrite(pinB, 0);
  }
  void backward(float speed) {
    int trueSpeed = 255 * speed;
    Serial.print("Backwards: ");
    ledcAnalogWrite(pinA, 0);
    ledcAnalogWrite(pinB, trueSpeed);
  }

  void drive(float speed) {
    if (speed > 0.1) {
      forward(speed);
    } else if (speed < -0.1) {
      backward(speed * -1);
    } else {
      stop();
    }
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

typedef struct struct_message {
  DriveType mode;
  ArmPosition armPos;
  float leftX;
  float leftY;
  float rightX;
  float rightY;
  int horn;
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
Servo arm;

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
  pinMode(BUZZER_PIN, OUTPUT);

  ledcAttach(LED_R, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcAttach(LED_G, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcAttach(LED_B, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);

  left.init(M1A, M1B, false);
  right.init(M2A, M2B, false);

  arm.attach(SERVO_PIN);
}

int droppedPackets = 0;
bool firstPacketRecieved = false;
ArmPosition armPos = REST;
ArmPosition lastRecvdArmPos = REST;

int i = 0;
int d = 5;
bool firstRun = true;

void loop() {

  if (newData) {
    // do things
    showColor(0, 0.5, 0.4);
    droppedPackets = 0;
    firstRun = false;

    if (recvData.horn) {
      digitalWrite(BUZZER_PIN, HIGH);
    } else {
      digitalWrite(BUZZER_PIN, LOW);
    }

    float xSpeed, zRot, leftSpeed, rightSpeed, greaterInput, lesserInput, saturatedInput;
    switch (recvData.mode) {
      case ARCADE:
        // handle arcade driving

        xSpeed = recvData.leftY;
        zRot = recvData.rightX;

        leftSpeed = xSpeed - zRot;
        rightSpeed = xSpeed + zRot;

        greaterInput = max(abs(xSpeed), abs(zRot));
        lesserInput = min(abs(xSpeed), abs(zRot));

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

    switch (recvData.armPos) {
      case UP:
        arm.write(0);
        break;
      case DOWN:
        arm.write(180);
        break;
      case REST:
        arm.write(82);
        break;
    }


    newData = false;
  } else {
    droppedPackets++;
  }
  if (droppedPackets > 5 && !firstRun) {
    left.stop();
    right.stop();
    arm.release();
    showColor(1.0, 0.0, 0.0, i);
    digitalWrite(BUZZER_PIN, LOW);
  }
  if (firstRun) {
    showColor(1, 1, 0);
  }
  i += d;
  if (i >= 255 || i <= 0) {
    d = -d;
  }
  delay(50);
}