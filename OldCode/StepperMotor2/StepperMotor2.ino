#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *stepMotor = AFMS.getMotor(4);

boolean shouldPullLeaves = false;

void setup() {
  AFMS.begin();  
  Serial.begin(9600);
stepMotor->setSpeed(55);
  shouldPullLeaves = true;
}

void loop() {
 pullLeaves(1300);
}

void pullLeaves(float delayTime) {
  stepMotor->run(FORWARD);
  delay(delayTime);
  stepMotor->run(RELEASE);
  delay(1000);
  stepMotor->run(BACKWARD);
  delay(delayTime);
  stepMotor->run(RELEASE);
  delay(1000);
}
