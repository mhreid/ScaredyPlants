const int front = A13;
const int left = A11;
const int right = A15;
int x = 0;
int y = 0;
float xmult = 0;
float ymult = 0;

const int threshold = 10; // Lighting threshold
const int dspeed = 80;
const int xcal = 12; // Need to calibrate xcal and ycal under the normal lighting condition
const int ycal = 0; 
const float forwardbuffer = .2;

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

//includes the motorshielf required libraries
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
//Creates the motorshield object
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);


void setup() {
  AFMS.begin();
  leftMotor->setSpeed(dspeed);
  rightMotor->setSpeed(dspeed);
  Serial.begin(9600);
}

void loop() {
  y = analogRead(front) - (analogRead(left) + analogRead(right))/2 + ycal;
  x = analogRead(left) - analogRead(right) + xcal;
  // positive y = front detected (move backward), positive x = left detected (move right)
  if (abs(x) > threshold or abs(y) > threshold) {
    // Scale down x and y
    xmult = abs(x)/100.0;
    ymult = abs(y)/50.0;

    // xmult is from 0 to 1
    if (xmult > 1){
      xmult = 1;
    }

    // ymult is from 0.2 to 1
    ymult += forwardbuffer;
    if (ymult > 1){
      ymult = 1;
    }

    // speeds are from 0 to dspeed
    int turnspeed = dspeed * ymult * (1 - xmult); // turnspeed <= forwardspeed
    int forwardspeed = dspeed * ymult;
    x = 10
   
    runCommand(forwardspeed, turnspeed);
  } else {
    stopCommand();
  }
  printValues();
}

// Helper function to set the wheels' speeds
void runCommand(int forwardspeed, int turnspeed) {
  if (x > 0){
    // Turn left
    rightMotor->setSpeed(forwardspeed);
    leftMotor->setSpeed(turnspeed);
  } else {
    // Turn right
    rightMotor->setSpeed(turnspeed);
    leftMotor->setSpeed(forwardspeed);    
  }
  leftMotor->run(y > 0 ? FORWARD : BACKWARD); // Conditional tenary operator
  rightMotor->run(y > 0 ? FORWARD : BACKWARD);
}

// Helper function to stop both wheels
void stopCommand() {
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

// Helper function to print whatever values we want
void printValues() {
  Serial.print(x);
  Serial.print("\t");
  Serial.println(y);
}
