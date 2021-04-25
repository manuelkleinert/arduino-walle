#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Arduino_ST7789_Fast.h>
//#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>             // Arduino SPI library

// #define TFT_CS    8
#define TFT_DC    9
#define TFT_RST   10 
#define SCR_WD   240
#define SCR_HT   240

Arduino_ST7789 tft = Arduino_ST7789(TFT_DC, TFT_RST);
// Adafruit_ST7789 tft = Adafruit_ST7789(TFT_DC, TFT_RST);
float p = 3.1415926;


int servoSwitchPin = 2;

int motorPinA = 3;
int motorPinDirA = 4;

int motorPinB = 5;
int motorPinDirB = 6;

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






void testlines(uint16_t color) {
  tft.fillScreen(BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawLine(0, 0, x, tft.height()-1, color);
    delay(0);
  }
  for (int16_t y=0; y < tft.height(); y+=6) {
    tft.drawLine(0, 0, tft.width()-1, y, color);
    delay(0);
  }
 
  tft.fillScreen(BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawLine(tft.width()-1, 0, x, tft.height()-1, color);
    delay(0);
  }
  for (int16_t y=0; y < tft.height(); y+=6) {
    tft.drawLine(tft.width()-1, 0, 0, y, color);
    delay(0);
  }
 
  tft.fillScreen(BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawLine(0, tft.height()-1, x, 0, color);
    delay(0);
  }
  for (int16_t y=0; y < tft.height(); y+=6) {
    tft.drawLine(0, tft.height()-1, tft.width()-1, y, color);
    delay(0);
  }
 
  tft.fillScreen(BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawLine(tft.width()-1, tft.height()-1, x, 0, color);
    delay(0);
  }
  for (int16_t y=0; y < tft.height(); y+=6) {
    tft.drawLine(tft.width()-1, tft.height()-1, 0, y, color);
    delay(0);
  }
}
 
void testdrawtext(char *text, uint16_t color) {
  tft.setCursor(0, 0);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}
 
void testfastlines(uint16_t color1, uint16_t color2) {
  tft.fillScreen(BLACK);
  for (int16_t y=0; y < tft.height(); y+=5) {
    tft.drawFastHLine(0, y, tft.width(), color1);
  }
  for (int16_t x=0; x < tft.width(); x+=5) {
    tft.drawFastVLine(x, 0, tft.height(), color2);
  }
}
 
void testdrawrects(uint16_t color) {
  tft.fillScreen(BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, color);
  }
}
 
void testfillrects(uint16_t color1, uint16_t color2) {
  tft.fillScreen(BLACK);
  for (int16_t x=tft.width()-1; x > 6; x-=6) {
    tft.fillRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, color1);
    tft.drawRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, color2);
  }
}
 
void testfillcircles(uint8_t radius, uint16_t color) {
  for (int16_t x=radius; x < tft.width(); x+=radius*2) {
    for (int16_t y=radius; y < tft.height(); y+=radius*2) {
      tft.fillCircle(x, y, radius, color);
    }
  }
}
 
void testdrawcircles(uint8_t radius, uint16_t color) {
  for (int16_t x=0; x < tft.width()+radius; x+=radius*2) {
    for (int16_t y=0; y < tft.height()+radius; y+=radius*2) {
      tft.drawCircle(x, y, radius, color);
    }
  }
}
 
void testtriangles() {
  tft.fillScreen(BLACK);
  int color = 0xF800;
  int t;
  int w = tft.width()/2;
  int x = tft.height()-1;
  int y = 0;
  int z = tft.width();
  for(t = 0 ; t <= 15; t++) {
    tft.drawTriangle(w, y, y, x, z, x, color);
    x-=4;
    y+=4;
    z-=4;
    color+=100;
  }
}
 
void testroundrects() {
  tft.fillScreen(BLACK);
  int color = 100;
  int i;
  int t;
  for(t = 0 ; t <= 4; t+=1) {
    int x = 0;
    int y = 0;
    int w = tft.width()-2;
    int h = tft.height()-2;
    for(i = 0 ; i <= 16; i+=1) {
      tft.drawRoundRect(x, y, w, h, 5, color);
      x+=2;
      y+=3;
      w-=4;
      h-=6;
      color+=1100;
    }
    color+=100;
  }
}
 
void tftPrintTest() {
  tft.setTextWrap(false);
  tft.fillScreen(BLACK);
  tft.setCursor(0, 30);
  tft.setTextColor(RED);
  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(YELLOW);
  tft.setTextSize(2);
  tft.println("Hello World!");
  tft.setTextColor(GREEN);
  tft.setTextSize(3);
  tft.println("Hello World!");
  tft.setTextColor(BLUE);
  tft.setTextSize(4);
  tft.print(1234.567);
  delay(1500);
  tft.setCursor(0, 0);
  tft.fillScreen(BLACK);
  tft.setTextColor(WHITE);
  tft.setTextSize(0);
  tft.println("Hello World!");
  tft.setTextSize(1);
  tft.setTextColor(GREEN);
  tft.print(p, 6);
  tft.println(" Want pi?");
  tft.println(" ");
  tft.print(8675309, HEX); // print 8,675,309 out in HEX!
  tft.println(" Print HEX!");
  tft.println(" ");
  tft.setTextColor(WHITE);
  tft.println("Sketch has been");
  tft.println("running for: ");
  tft.setTextColor(MAGENTA);
  tft.print(millis() / 1000);
  tft.setTextColor(WHITE);
  tft.print(" seconds.");
}
 
void mediabuttons() {
  // play
  tft.fillScreen(BLACK);
  tft.fillRoundRect(25, 10, 78, 60, 8, WHITE);
  tft.fillTriangle(42, 20, 42, 60, 90, 40, RED);
  delay(500);
  // pause
  tft.fillRoundRect(25, 90, 78, 60, 8, WHITE);
  tft.fillRoundRect(39, 98, 20, 45, 5, GREEN);
  tft.fillRoundRect(69, 98, 20, 45, 5, GREEN);
  delay(500);
  // play color
  tft.fillTriangle(42, 20, 42, 60, 90, 40, BLUE);
  delay(50);
  // pause color
  tft.fillRoundRect(39, 98, 20, 45, 5, RED);
  tft.fillRoundRect(69, 98, 20, 45, 5, RED);
  // play color
  tft.fillTriangle(42, 20, 42, 60, 90, 40, GREEN);
}





void setup() {

  // pinMode(servoSwitchPin, OUTPUT);
  // pinMode(motorPinDirA, OUTPUT);
  // pinMode(motorPinDirB, OUTPUT);
  
  Serial.begin(115200);
  Serial.setTimeout(100);

  while (!Serial) continue;

  // tft.init(240, 240, SPI_MODE1); 
  tft.init(SCR_WD, SCR_HT);
  tft.setRotation(4);
 
  Serial.println(F("Initialized"));
 
  uint16_t time = millis();
  tft.fillScreen(BLACK);
  time = millis() - time;
 
  Serial.println(time, DEC);
  delay(500);
 
  // large block of text
  tft.fillScreen(BLACK);
  testdrawtext("Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur adipiscing ante sed nibh tincidunt feugiat. Maecenas enim massa, fringilla sed malesuada et, malesuada sit amet turpis. Sed porttitor neque ut ante pretium vitae malesuada nunc bibendum. Nullam aliquet ultrices massa eu hendrerit. Ut sed nisi lorem. In vestibulum purus a tortor imperdiet posuere. ", WHITE);
  delay(1000);
 
  // tft print function!
  tftPrintTest();
  delay(4000);
 
  // a single pixel
  tft.drawPixel(tft.width()/2, tft.height()/2, GREEN);
  delay(500);
 
  // line draw test
  testlines(YELLOW);
  delay(500);
 
  // optimized lines
  testfastlines(RED, BLUE);
  delay(500);
 
  testdrawrects(GREEN);
  delay(500);
 
  testfillrects(YELLOW, MAGENTA);
  delay(500);
 
  tft.fillScreen(BLACK);
  testfillcircles(10, BLUE);
  testdrawcircles(10, WHITE);
  delay(500);
 
  testroundrects();
  delay(500);
 
  testtriangles();
  delay(500);
 
  mediabuttons();
  delay(500);
 
  Serial.println("done");
  delay(1000);


  // // Motor direction
  // digitalWrite(motorPinDirA, HIGH);
  // digitalWrite(motorPinDirB, LOW);

  // pwm.begin();
  // pwm.setOscillatorFrequency(27000000);
  // pwm.setPWMFreq(50);
  
  // delay(500);
  // setServos();
  // delay(500);

  // Serial.println("<Wall-E is ready>");
  // Serial.flush();
}

void loop() {
  // setServos();
  // readSerial();
  // setMotor();
}
