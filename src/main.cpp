#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <ArduinoJson.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_GFX.h>
#include <Arduino_ST7789_Fast.h>

#define TFT_CS    -1
#define TFT_DC    48
#define TFT_RST   49
#define SCR_WD   240
#define SCR_HT   240

Arduino_ST7789 tft = Arduino_ST7789(TFT_DC, TFT_RST);

int servoSwitchPin = 2;

int motorPinA = 3;
int motorPinDirA = 12;
int motorPinBreakA = 9;

int motorPinB = 11;
int motorPinDirB = 13;
int motorPinBreakB = 8;

int servoDefault = 300;
int servoMax = 500;
int servoMin = 100;

int ledRedPin = 43;
int ledRedBrightness = 50;
int ledRedFadeAmount = 1;

int ledGreenPin = 45;
int ledGreenBrightness = 50;
int ledGreenFadeAmount = 1;

int ledBluePin = 47;
int ledBlueBrightness = 50;
int ledBlueFadeAmount = 1;

int tftUpdateIndex = 0;
int tftSunFlashIndex = 0;

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

int motorDelayIndex = 50;
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
          Serial.println("!!! Invalid input!");
          Serial.println(readString);
          break;
      case DeserializationError::NoMemory:
          Serial.println("!!! Not enough memory");
          Serial.println(readString);
          break;
      default:
          Serial.println("!!! Deserialization failed");
          Serial.println(readString);
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
    digitalWrite(motorPinDirA, motorDirection[0]);
    digitalWrite(motorPinDirB, motorDirection[1]);
    analogWrite(motorPinA, motorSpeed[0]);
    analogWrite(motorPinB, motorSpeed[1]);
    motorDelayIndex --;
    delay(10);
  } else {
    analogWrite(motorPinA, 0);
    analogWrite(motorPinB, 0);
    delay(10);
  }
}

void setLed() {
  analogWrite(ledRedPin, ledRedBrightness);
  ledRedBrightness = ledRedBrightness + ledRedFadeAmount;
  if (ledRedBrightness <= 20 || ledRedBrightness >= 100) {
    ledRedFadeAmount = -ledRedFadeAmount;
  }
}

void setTft() {
  if (tftUpdateIndex%10 == 0) {
    tft.setTextColor(YELLOW);
    tft.setTextWrap(true);
    tft.setCursor(15, 10);
    tft.setTextSize(2);
    tft.print("SOLAR CHARGE LEVEL");

    if (tftSunFlashIndex < 10) {
      for (int i = 0; i < 5; i++) {
        tft.drawLine(58 + i, 40, 58 + i, 130, YELLOW);
        tft.drawLine(15, 83 + i, 105, 83 + i, YELLOW);
      }
      tft.fillCircle(60, 85, 13, BLACK);
    } else {
      tft.fillRect(5, 30, 110, 110, BLACK);
    }

    for (int i = 14; i < 30; i++) {
      uint16_t circleColor = i < 20 ? YELLOW : BLACK;
      tft.drawCircle(60, 85, i, circleColor);
    }

    tftSunFlashIndex ++;

    if (tftSunFlashIndex > 2) {
      tftSunFlashIndex = 0;
    }
    

    for (int i = 0; i <= 8; i++) {
      uint16_t lineColor = i < 5 ? GREY : YELLOW;
      int lineBold = i == 8 ? 15 : 6;  
      tft.drawFastHLine(122, (50 + (i * 20)), 111, lineColor);
      tft.drawFastHLine(121, (51 + (i * 20)), 113, lineColor);
      tft.fillRect(120, (52 + (i * 20)), 115, lineBold, lineColor);
      tft.drawFastHLine(121, (52 + lineBold + (i * 20)), 113, lineColor);
      tft.drawFastHLine(122, (53 + lineBold + (i * 20)), 111, lineColor);
    }
  }
  
  tftUpdateIndex ++;
}

void setup() {

  pinMode(servoSwitchPin, OUTPUT);
  pinMode(motorPinDirA, OUTPUT);
  pinMode(motorPinDirB, OUTPUT);
  
  Serial.begin(115200);
  Serial.setTimeout(100);

  while (!Serial) continue;

    // Display
  tft.init(240, 240);
  tft.setRotation(4);
  tft.fillScreen(BLACK);
  tft.setTextColor(WHITE);
  tft.setTextWrap(true);
  tft.setCursor(5, 5);
  tft.setTextSize(2);
  tft.print("load ...");
  tft.setCursor(30, 50);
  tft.setTextSize(5);
  tft.print("Wall-E");

  // LED 
  analogWrite (ledRedPin, 200);  
  analogWrite (ledBluePin, 0);  
  analogWrite (ledGreenPin, 200);

  delay(1000);

  analogWrite (ledRedPin, 0);  
  analogWrite (ledBluePin, 0);  
  analogWrite (ledGreenPin, 200);

  delay(1000);

  analogWrite (ledRedPin, 0);  
  analogWrite (ledBluePin, 200);  
  analogWrite (ledGreenPin, 0);

  delay(1000);

  analogWrite (ledBluePin, 0);  

  // Motor direction
  digitalWrite(motorPinDirA, HIGH);
  digitalWrite(motorPinDirB, LOW);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  
  delay(500);
  setServos();
  delay(2000);

  Serial.println("<Wall-E is ready>");
  Serial.flush();
  tft.fillScreen(BLACK);
}

void loop() {
  setServos();
  readSerial();
  setMotor();
  setLed();
  setTft();
}
