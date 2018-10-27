const int front = A0;
const int left = A1;
const int right = A2;
int x = 0;
int y = 0;

const int threshold = 40;
const int dspeed = 50;

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
//includes the motorshielf required libraies
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
//Creates the motrshield object


Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);
//Creates motor objects, parameter is the port it is connected to (M1 = 1, M2 = 2, M3 = 3, M4 = 4)


void setup() {
  // put your setup code here, to run once:
   AFMS.begin();
  leftMotor->setSpeed(50);
  rightMotor->setSpeed(50);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  y = analogRead(front) - (analogRead(left) + analogRead(right))/2;
  x = analogRead(left) - analogRead(right);
  //positive y = forward, positive x = left
  if(x > threshold or y > threshold or x * -1 > threshold or y * -1 > threshold){
    if(x < 0){
      rightMotor->setSpeed(dspeed / 3);
      leftMotor->setSpeed(dspeed);
    } else {
      rightMotor->setSpeed(dspeed);
      leftMotor->setSpeed(dspeed / 3);    
    }

    if(y > 0){
      leftMotor->run(FORWARD);
      rightMotor->run(FORWARD);
    } else {
      leftMotor->run(BACKWARD);
      rightMotor->run(BACKWARD);
    }
  } else {
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
  }
  Serial.print(x);
  Serial.print(" , ");
  Serial.println(y);
}
