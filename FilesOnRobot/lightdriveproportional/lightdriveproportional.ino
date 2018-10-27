const int front = A2;
const int left = A1;
const int right = A0;
int x = 0;
int y = 0;
float xmult = 0;
float ymult = 0;

const int threshold = 10;
const int m = 100;
const int dspeed = 40;
const int xcal = -44;
const int ycal = -38;

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
  y = analogRead(front) - (analogRead(left) + analogRead(right))/2 + ycal;
  x = analogRead(left) - analogRead(right) + xcal;
  //positive y = forward, positive x = left
  if(abs(x) > threshold or abs(y) > threshold){
    xmult = float(abs(x))/100.0;
    ymult = float(abs(y))/ 50.0;
    if(xmult > 1){
      xmult = 1;
    }
    float forwardbuffer = .2;

    if(ymult > 1 - forwardbuffer){
      ymult = 1 - forwardbuffer;
    }
    int turnspeed = dspeed * (ymult + forwardbuffer) * (1-xmult);
    int forwardspeed = dspeed * (ymult + forwardbuffer);
    if (turnspeed > 50)
      turnspeed = 50;
    if (forwardspeed > 50)
      forwardspeed = 50;
    if (x > 0){
      rightMotor->setSpeed(forwardspeed);
      leftMotor->setSpeed(turnspeed);
    } else {
      rightMotor->setSpeed(turnspeed);
      leftMotor->setSpeed(forwardspeed);    
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
  Serial.print(String(x));
  Serial.print(" , ");
  Serial.println(String(y));
}
