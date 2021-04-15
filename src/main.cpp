#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <Adafruit_PWMServoDriver.h>

int backA = 12;
int backB = 13;

int servoDefault = 300;
int servoMax = 500;
int servoMin = 100;

/*
  Servos:
  0 - Arm R
  1 - Arm L

  4 - Neck down
  5 - Neck middle
  6 - Turn Head

  8 - Eye right
  9 - Eye left

  12 - Front flap

                                     0    1    2    3    4    5    6    7    8    9    10   11   12   13   14   15
*/
int servoMinArray[16]            = {100, 100, 100, 100, 150, 140, 120, 100, 280, 230, 100, 100, 100, 100, 100, 100};
int servoMaxArray[16]            = {500, 500, 500, 500, 500, 400, 500, 500, 405, 320, 500, 500, 500, 500, 500, 500};

int servoPositionArray[16]       = {300, 300, 300, 300, 100, 100, 270, 300, 353, 248, 300, 300, 300, 300, 300, 300};
int servoTargetPositionArray[16] = {300, 300, 300, 300, 100, 100, 270, 300, 353, 248, 300, 300, 300, 300, 300, 300};

float servoSpeedArray[16]        = {  1,   1,   1,   1,   1,   1,   1,   1,   4,   4,   1,   1,   1,   1,   1,   1};

int motorDelayIndex = 20;
int motorDirection[2] = {HIGH, LOW};
int motorSpeed[2] = {0, 0};

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
DeserializationError err;

DynamicJsonDocument doc(1024);
String readString;

// our servo # counter
uint8_t servonum = 0;

boolean readSerial() {
  if (Serial.available()) {
    readString = "";

    Serial.println("====== Serial read ======");

    while (Serial.available()) {
      readString += Serial.readStringUntil('\n');

      Serial.println("====== read ======");
      Serial.println(readString);
    }

    Serial.println("====== SET ======");
    Serial.println(readString);
    
    err = deserializeJson(doc, readString);

    Serial.println(err.code());


    switch (err.code()) {
      case DeserializationError::Ok:
        if (isDigit(doc["pin"]) && isDigit(doc["pos"])) {
          int pin = (int)doc["pin"];
          int position = (int)doc["pos"];
          float speed = (float)doc["speed"];
          servoTargetPositionArray[pin] = position ? position : 300;
          servoSpeedArray[pin] = speed ? speed: 1;
        } else if (doc["dir"] != NULL) {
          motorDirection[0] = (int)doc["dir"][0] ? HIGH : LOW;
          motorDirection[1] = (int)doc["dir"][1] ? LOW : HIGH;
          motorSpeed[0] = (int)doc["speed"][0];
          motorSpeed[1] = (int)doc["speed"][1];
          motorDelayIndex = 20;
        }

        return true;
        break;
      case DeserializationError::InvalidInput:
          Serial.print("!!! Invalid input!");
          break;
      case DeserializationError::NoMemory:
          Serial.print("!!! Not enough memory");
          break;
      default:
          Serial.print("!!! Deserialization failed");
          break;
    }
  }

  return false;
}

void setServos() {
  for (int i = 0; i < 16; i++) {
    if (servoTargetPositionArray[i] != servoPositionArray[i] &&
      (servoMaxArray[i] >= servoPositionArray[i] || servoMaxArray[i] >= servoTargetPositionArray[i]) 
       && (servoMinArray[i] <= servoPositionArray[i] || servoMinArray[i] <= servoTargetPositionArray[i])) {

      if (servoPositionArray[i] < servoTargetPositionArray[i]) {
        servoPositionArray[i] = (int) (servoPositionArray[i] + servoSpeedArray[i]);
      } else {
        servoPositionArray[i] = (int) (servoPositionArray[i] - servoSpeedArray[i]);
      }

      pwm.setPWM(i, 0, servoPositionArray[i]);
    }
  }
}

void setMotor() {
  if (motorDelayIndex > 0) {
    digitalWrite(backA, motorDirection[0]);
    digitalWrite(backB, motorDirection[1]);
    analogWrite(3, motorSpeed[0]);
    analogWrite(11, motorSpeed[1]);
    motorDelayIndex --;
  } else {
    analogWrite(3, 0);
    analogWrite(11, 0);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  while (!Serial) continue;

  pinMode(backA, OUTPUT);
  pinMode(backB, OUTPUT);

  // Motor direction
  digitalWrite(backA, HIGH);
  digitalWrite(backB, LOW);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);

  
  delay(500);
  setServos();
  delay(500);

  Serial.println("<Wall-E is ready>");
  Serial.flush();
}

void loop() {
  setServos();
  readSerial();
  setMotor();
}
