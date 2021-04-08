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
int servoMinArray[16]            = {100, 100, 100, 100, 150, 140,  50, 100, 200, 200, 100, 100, 100, 100, 100, 100};
int servoMaxArray[16]            = {500, 500, 500, 500, 500, 420, 500, 500, 400, 400, 500, 500, 500, 500, 500, 500};

int servoPositionArray[16]       = {300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300};
int servoTargetPositionArray[16] = {300, 300, 300, 300, 100, 100, 270, 300, 300, 300, 300, 300, 300, 300, 300, 300};

float servoSpeedArray[16] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

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
          int pin = (int)doc["pin"];
          int position = (int)doc["pos"];
          float speed = (float)doc["speed"];
          servoTargetPositionArray[pin] = position ? position : 300;
          servoSpeedArray[pin] = speed ? speed: 1;
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
    }

    pwm.setPWM(i, 0, servoPositionArray[i]);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
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

  // pwm.setPWM(12, 0, 300);
  // pwm.setPWM(13, 0, 300);

  // delay(5000);

  // pwm.setPWM(12, 0, 400);
  // pwm.setPWM(13, 0, 200);

  // delay(5000);

/*    // Drive each servo one at a time using setPWM()
  Serial.println(servonum);
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(servonum, 0, pulselen);
  }

  delay(500);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(servonum, 0, pulselen);
  }

  delay(500);

  // Drive each servo one at a time using writeMicroseconds(), it's not precise due to calculation rounding!
  // The writeMicroseconds() function is used to mimic the Arduino Servo library writeMicroseconds() behavior. 
  for (uint16_t microsec = USMIN; microsec < USMAX; microsec++) {
    pwm.writeMicroseconds(servonum, microsec);
  }

  delay(500);
  for (uint16_t microsec = USMAX; microsec > USMIN; microsec--) {
    pwm.writeMicroseconds(servonum, microsec);
  }

  delay(500);

  servonum++;
  if (servonum > 12) servonum = 0; // Testing the first 8 servo channels
 */

  // forward @ full speed
  // digitalWrite(mm, HIGH); //Establishes forward direction of Channel A
  // digitalWrite(mb, LOW);   //Disengage the Brake for Channel A
  // analogWrite(3, 255);   //Spins the motor on Channel A at full speed
  
  // delay(3000);
  

  // delay(1000);
  
  // digitalWrite(backA, HIGH);
  // digitalWrite(backB, LOW);

  // analogWrite(3, 120);   //Spins the motor on Channel A at half speed
  // analogWrite(11, 120);
  
  // delay(200);

  // digitalWrite(backB, HIGH);
  // digitalWrite(backA, LOW);

  // analogWrite(3, 120);   //Spins the motor on Channel A at half speed
  // analogWrite(11, 120);
  
  // delay(200);

  //   digitalWrite(backA, HIGH);
  // digitalWrite(backB, LOW);

  // analogWrite(3, 120);   //Spins the motor on Channel A at half speed
  // analogWrite(11, 120);
  
  // delay(200);

  // digitalWrite(backB, HIGH);
  // digitalWrite(backA, LOW);

  // analogWrite(3, 120);   //Spins the motor on Channel A at half speed
  // analogWrite(11, 120);
  
  // delay(200);

  // digitalWrite(backA, HIGH);
  // digitalWrite(backB, LOW);

  // analogWrite(3, 120);   //Spins the motor on Channel A at half speed
  // analogWrite(11, 120);
  
  // delay(200);

  // digitalWrite(backB, HIGH);
  // digitalWrite(backA, LOW);

  // analogWrite(3, 120);   //Spins the motor on Channel A at half speed
  // analogWrite(11, 120);
  
  // delay(200);
 
  // analogWrite(3, 0);
  // analogWrite(11, 0); 
  
  // delay(1000);

  // digitalWrite(backB, LOW);
  // digitalWrite(backA, LOW);

  // analogWrite(3, 120);
  // analogWrite(11, 120);


  // delay(1000);

  // analogWrite(3, 0);
  // analogWrite(11, 0); 
  
  // delay(1000);


  // digitalWrite(backB, HIGH);
  // digitalWrite(backA, HIGH);

  // analogWrite(3, 120);
  // analogWrite(11, 120);


  // delay(1000);


  // analogWrite(3, 0);
  // analogWrite(11, 0); 


}
