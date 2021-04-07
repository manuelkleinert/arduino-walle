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
int servoMinArray[16] = {100, 100, 100, 100, 150, 140, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
int servoMaxArray[16] = {500, 500, 500, 500, 500, 420, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500};

int servoPositionArray[16] = {300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300};
int servoTargetPositionArray[16] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

int servoSpeedArray[16] = {};

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
DeserializationError error;

DynamicJsonDocument doc(1024);
String readString;

// our servo # counter
uint8_t servonum = 0;

boolean readSerial() {
  if (Serial.available()) {

    readString = "";
    while (Serial.available()) {
      // delay(1);  //delay to allow byte to arrive in input buffer
      
      readString += Serial.readStringUntil('\n');
      // readString += (char) Serial.read();

      //readString += Serial.readString();

      // Serial.println("====");
      // Serial.println(readString);
      // Serial.println("====");


      // Serial.println(Serial.readString());

      // readString += Serial.readString();
      // Serial.println(readString);
    }

    Serial.println("Arduino Serial: START::::");
    Serial.println(readString);
    Serial.println(" ::: END");
    
    DeserializationError error = deserializeJson(doc, readString);

    switch (error.code()) {
      case DeserializationError::Ok:
          Serial.println("OK <<<<<<<<<<<<<<<<<<<<<<<<");
          String test = doc["servos"];
          Serial.println(test);
          // int pin = doc["sPin"];
          // int position = doc["Pos"];
          // int speed = doc["Speed"];
          // servoTargetPositionArray[pin] = position ? position : 300;
          // servoSpeedArray[pin] = speed ? speed: 1;
          return true;
          break;
      case DeserializationError::InvalidInput:
          Serial.print("Invalid input!");
          break;
      case DeserializationError::NoMemory:
          Serial.print("Not enough memory");
          break;
      default:
          Serial.print("Deserialization failed");
          break;
    }
  }

  return false;
}

void setServos() {
  for (int i = 0; i < 16; i++) {
    if(servoTargetPositionArray[i] == servoPositionArray[i]) {
      continue;
    }

    if ((servoMaxArray[i] <= servoPositionArray[i] && servoTargetPositionArray[i] >= servoMaxArray[i]) 
    || (servoMinArray[i] >= servoPositionArray[i] && servoTargetPositionArray[i] <= servoMinArray[i])) {
      continue;
    }

    if (servoPositionArray[i] < servoTargetPositionArray[i]) {
      servoPositionArray[i] += servoSpeedArray[i];
    } else {
      servoPositionArray[i] -= servoSpeedArray[i];
    }
    pwm.setPWM(i, 0, servoPositionArray[i]);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
  while (!Serial) continue;

  pinMode(backA, OUTPUT);
  pinMode(backB, OUTPUT);

  // Motor direction
  digitalWrite(backA, HIGH);
  digitalWrite(backB, LOW);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(60);

  setServos();

  Serial.println("<Arduino is ready>");
  delay(500);
  Serial.flush();
}

void loop() {
  // setServos();
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
