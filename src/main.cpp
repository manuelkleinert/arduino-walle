#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <Adafruit_PWMServoDriver.h>

int backA = 12;
int backB = 13;

int servoDefault = 0;
int servoMax = 540;
int servoMin = 50;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;

String incomingString = "";

DynamicJsonDocument doc(1024);

void setup() {
  Serial.begin(115200);

  pinMode (backA, OUTPUT);
  pinMode (backB, OUTPUT);

  digitalWrite (backA, HIGH);
  digitalWrite (backB, LOW);

  pwm.begin();

  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  pwm.setPWM(4, 0, 300);
  pwm.setPWM(5, 0, 300);

  pwm.setPWM(6, 0, 300);
  pwm.setPWM(7, 0, 300);

  Serial.println("<Arduino is ready>");

  delay(500);
}

void loop() {
  Serial.print(".");

   // send data only when you receive data:
  if (Serial.available()) {
    // read the incoming byte:
    incomingString = "";
    while(Serial.available())
    {
      incomingString+= String((char)Serial.read());
    }

    // say what you got:
    Serial.print("START:: ");
    Serial.print(incomingString);

    deserializeJson(doc, incomingString);
    JsonObject obj = doc.as<JsonObject>();
    int servoId = obj["Servo"];
    int position = obj["Pos"];

    Serial.println(" ::END");
    Serial.flush();

    if (servoMax >= position and servoMin <= position) {
      pwm.setPWM(servoId, 0, position);
    }
  } else {
    Serial.println("Serial Available false");
  }

  delay(200);

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