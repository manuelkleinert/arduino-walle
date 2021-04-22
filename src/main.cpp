#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <Adafruit_PWMServoDriver.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_ST7789.h>


// int  TFT_DC = 7;     // TFT DC  pin is connected to NodeMCU pin D1 (GPIO5)
// int  TFT_RST = 6;     // TFT RST pin is connected to NodeMCU pin D2 (GPIO4)
// int  TFT_CS = 100;   // TFT CS  pin is connected to NodeMCU pin D8 (GPIO15)

// Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);


int servoSwitchPin = 2;
int motorPinA = 12;
int motorPinB = 13;

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
int servoMinArray[16]            = {200, 200, 100, 100, 150, 140, 120, 100, 280, 230, 100, 100, 100, 100, 100, 100};
int servoMaxArray[16]            = {400, 400, 500, 500, 500, 400, 500, 500, 405, 320, 500, 500, 500, 500, 500, 500};

int servoPositionArray[16]       = {300, 300, 300, 300, 100, 100, 270, 300, 353, 248, 300, 300, 300, 300, 300, 300};
int servoTargetPositionArray[16] = {300, 300, 300, 300, 100, 100, 270, 300, 353, 248, 300, 300, 300, 300, 300, 300};

float servoSpeedArray[16]        = {  1,   1,   1,   1,   1,   1,   1,   1,   4,   4,   1,   1,   1,   1,   1,   1};
int servoDelayIndex = 200;

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

    while (Serial.available()) {
      readString += Serial.readStringUntil('\n');
    }
    
    err = deserializeJson(doc, readString);

    switch (err.code()) {
      case DeserializationError::Ok:
        if (doc["pin"] != "" && doc["pos"]) {
          int pin = (int)doc["pin"];
          int position = (int)doc["pos"];
          float speed = (float)doc["speed"];
          servoTargetPositionArray[pin] = position ? position : 300;
          servoSpeedArray[pin] = speed ? speed: 1;
          servoDelayIndex = 200;
        } else if (doc["dir"]) {
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

void switchServos() {
  if (servoDelayIndex > 0) {
    servoDelayIndex --;
    digitalWrite(servoSwitchPin, HIGH);    
  } else {
    digitalWrite(servoSwitchPin, LOW);
  }
}

void setServos() {
  switchServos();
  for (int i = 0; i < 16; i++) {
    if (servoTargetPositionArray[i] != servoPositionArray[i] &&
      (servoMaxArray[i] >= servoPositionArray[i] || servoMaxArray[i] >= servoTargetPositionArray[i]) 
       && (servoMinArray[i] <= servoPositionArray[i] || servoMinArray[i] <= servoTargetPositionArray[i])) {

      if (servoPositionArray[i] < servoTargetPositionArray[i]) {
        servoPositionArray[i] = (int) (servoPositionArray[i] + servoSpeedArray[i]);
      } else {
        servoPositionArray[i] = (int) (servoPositionArray[i] - servoSpeedArray[i]);
      }
    }

    if (servoDelayIndex > 0) {
      pwm.setPWM(i, 0, servoPositionArray[i]);
    }
  }
}

void setMotor() {
  if (motorDelayIndex > 0) {
    digitalWrite(motorPinA, motorDirection[0]);
    digitalWrite(motorPinB, motorDirection[1]);
    analogWrite(3, motorSpeed[0]);
    analogWrite(11, motorSpeed[1]);
    motorDelayIndex --;
  } else {
    analogWrite(3, 0);
    analogWrite(11, 0);
  }
}

// void testdrawtext(char *text, uint16_t color) {
//   tft.setCursor(0, 0);
//   tft.setTextColor(color);
//   tft.setTextWrap(true);
//   tft.print(text);
// }

void setup() {
  
  Serial.begin(115200);
  Serial.setTimeout(100);

  while (!Serial) continue;

  pinMode(servoSwitchPin, OUTPUT);
  pinMode(motorPinA, OUTPUT);
  pinMode(motorPinB, OUTPUT);

  // Motor direction
  digitalWrite(motorPinA, HIGH);
  digitalWrite(motorPinB, LOW);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  
  delay(500);
  setServos();
  delay(500);


  // if the display has CS pin try with SPI_MODE0
  // tft.init(240, 240, SPI_MODE2);    // Init ST7789 display 240x240 pixel
 
  // // if the screen is flipped, remove this command
  // tft.setRotation(2);

  // tft.fillScreen(ST77XX_BLACK);
  // testdrawtext("Wall-E", ST77XX_WHITE);


  Serial.println("<Wall-E is ready>");
  Serial.flush();
}

void loop() {
  setServos();
  readSerial();
  setMotor();
}
