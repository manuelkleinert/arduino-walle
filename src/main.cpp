#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <ArduinoJson.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_GFX.h>
#include <Arduino_ST7789_Fast.h>
#include <VL53L1X.h>

#define TFT_CS     -1
#define TFT_DC     48
#define TFT_RST    49
#define SCR_WD    240
#define SCR_HT    240

#define SER_SW     37

#define MOT_A       3
#define MOT_A_DIR  12
#define MOT_A_BR    9
#define MOT_B      11
#define MOT_B_DIR  13
#define MOT_B_BR    8

#define LED_RED     43
#define LED_GREEN   45
#define LED_BLUE    47

Arduino_ST7789 tft = Arduino_ST7789(TFT_DC, TFT_RST);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


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

int servoDefault = 300;
int servoMax = 500;
int servoMin = 100;

int ledRedBrightness = 200;
int ledRedFadeAmount = 2;

int ledGreenBrightness = 0;
int ledGreenFadeAmount = 2;

int ledBlueBrightness = 0;
int ledBlueFadeAmount = 2;

int tftUpdateIndex = 0;
int tftSunFlashIndex = 0;

uint8_t servonum = 0;

DynamicJsonDocument doc(1024);
String readString;

VL53L1X distFrontLeft;
DeserializationError err;

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

        } else if (doc["led"]) {

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


void l2cScanner() {
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      Serial.print(address < 16 ? "0" : "");
      Serial.println(address, HEX);
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      Serial.print(address < 16 ? "0" : "");
      Serial.println(address,HEX);
    }    
  }
  Serial.println(nDevices == 0 ? "No I2C devices found\n" : "done\n");
 
  delay(1000); 
}

void switchServos() {
  if (servoDelayIndex > 0) {
    servoDelayIndex --;
    digitalWrite(SER_SW, HIGH);    
  } else {
    digitalWrite(SER_SW, LOW);
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
    digitalWrite(MOT_A_DIR, motorDirection[0]);
    digitalWrite(MOT_B_DIR, motorDirection[1]);
    analogWrite(MOT_A, motorSpeed[0]);
    analogWrite(MOT_B, motorSpeed[1]);
    motorDelayIndex --;
    delay(10);
  } else {
    analogWrite(MOT_A, 0);
    analogWrite(MOT_B, 0);
    delay(10);
  }
}

void setLed() {
  analogWrite(LED_RED, ledRedBrightness);
  ledRedBrightness = ledRedBrightness + ledRedFadeAmount;

  if (ledRedBrightness <= 60 || ledRedBrightness >= 250) {
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
  pinMode(SER_SW, OUTPUT);
  pinMode(MOT_A_DIR, OUTPUT);
  pinMode(MOT_B_DIR, OUTPUT);
  
  Serial.begin(115200);
  Serial.setTimeout(100);
  while (!Serial) continue;

  Wire.begin();
  Wire.setClock(400000);

  l2cScanner();

  distFrontLeft.setTimeout(500);
  if (!distFrontLeft.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  
  distFrontLeft.setDistanceMode(VL53L1X::Long);
  distFrontLeft.setMeasurementTimingBudget(15000);
  distFrontLeft.startContinuous(15);

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
  analogWrite (LED_RED, 0);  
  analogWrite (LED_BLUE, 200);  
  analogWrite (LED_GREEN, 0);
  delay(500);
  
  // Servo
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

  Serial.println(String(distFrontLeft.read()));
}
