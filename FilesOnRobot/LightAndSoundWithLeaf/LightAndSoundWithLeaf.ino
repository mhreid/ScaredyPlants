/****************************************
  This code prioritized Sound.
  If the robot gets a sound input, a flag will 
  be set to "TRUE" and it will skip the light 
  sensing for a little while.
****************************************/
// Mic 1 @ 120 degrees connects to A9 (FRONT)
// Mic 2 @ 0 degrees connects to A13 (RIGHT)
// Mic 3 @ 240 degrees connects to A11 (LEFT)

// MOTOR STUFF
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

//includes the motorshield required libraries
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
//Creates the motorshield object
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);
Adafruit_DCMotor *leafMotor = AFMS.getMotor(4);


//SOUND STUFF
// Sets up Variables that will be used
const int sampleWindow = 100; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;
unsigned int sample2;
unsigned int sample3;
unsigned int angle_deg = 0;
int threshold_sound = 50;
unsigned long moveMillis = millis(); // Starts counting time for how long an angle has been given


//LIGHT STUFF
const int front = A4;
const int right = A15;
const int left = A2;
int x = 0;
int y = 0;
float xmult = 0;
float ymult = 0;

const int threshold_light = 10;
const int dspeed = 110;// default speed for wheels
const int frontCal = 0; // Need to calibrate xcal and ycal under the normal lighting condition
const int leftCal = 80;
const int rightCal = 30;
const float forwardbuffer = .2;

void setup()
{
  // MOTOR SHIELD SETUP
  AFMS.begin();
  leftMotor->setSpeed(dspeed);
  rightMotor->setSpeed(dspeed);
  leafMotor->setSpeed(55);
  Serial.begin(9600);
  angle_deg = 0;
}


void loop()
{
  senseSound(true);
  senseLight(false);
}


// TODO: Need to switch from Adafruit motor to a servo
// LEAF SHRINKING
void pullLeaves(float delayTime) {
  leafMotor->run(FORWARD);
  delay(delayTime);
  leafMotor->run(RELEASE);
  delay(1000);
  leafMotor->run(BACKWARD);
  delay(delayTime);
  leafMotor->run(RELEASE);
}


// SOUND SENSING
void senseSound(bool shouldPrint) {
  static unsigned long startMillis;  // Start of sample window
  unsigned int peakToPeak = 0;   // peak-to-peak level
  unsigned int peakToPeak2 = 0; // Mic 2
  unsigned int peakToPeak3 = 0; // Mic 3

  static unsigned int signalMax; // Mic 1
  static unsigned int signalMin;
  static unsigned int signalMax2; // Mic 2
  static unsigned int signalMin2;
  static unsigned int signalMax3; // Mic 3
  static unsigned int signalMin3;


  if (startMillis == 0) { // Re-initialize everything
    startMillis = millis();
    signalMax = 0;
    signalMax2 = 0;
    signalMax3 = 0;
    signalMin = 1024;
    signalMin2 = 1024;
    signalMin3 = 1024;
  }
  // collect data for 50 mS
  if (millis() - startMillis < sampleWindow)
  {
    sample = analogRead(A13);
    sample2 = analogRead(A9);
    sample3 = analogRead(A11);
    // Mic 1
    if (sample < 1024)  // toss out spurious readings
    {
      if (sample > signalMax)
      {
        signalMax = sample;  // save just the max levels
      }
      else if (sample < signalMin)
      {
        signalMin = sample;  // save just the min levels
      }
    }
    // Mic 2
    if (sample2 < 1024)  // toss out spurious readings
    {
      if (sample2 > signalMax2)
      {
        signalMax2 = sample2;  // save just the max levels
      }
      else if (sample2 < signalMin2)
      {
        signalMin2 = sample2;  // save just the min levels
      }
    }
    // Mic 3
    if (sample3 < 1024)  // toss out spurious readings
    {
      if (sample3 > signalMax3)
      {
        signalMax3 = sample3;  // save just the max levels
      }
      else if (sample3 < signalMin3)
      {
        signalMin3 = sample3;  // save just the min levels
      }
    }
  } else {
    peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
    double volts = 1500 * (peakToPeak * 5.0) / 1024;  // convert to volts
    // Mic 2
    peakToPeak2 = signalMax2 - signalMin2;  // max - min = peak-peak amplitude
    double volts2 = 1000 * (peakToPeak2 * 5.0) / 1024;  // convert to volts
    // Mic 3
    peakToPeak3 = signalMax3 - signalMin3;  // max - min = peak-peak amplitude
    double volts3 = 1000 * (peakToPeak3 * 5.0) / 1024;  // convert to volts

    // Maybe later, change to if(volts+volts2+volts3 > threshold_sound*3)
    if (volts > threshold_sound || volts2 > threshold_sound || volts3 > threshold_sound) {
      if (volts > volts2) {
        if (volts3 > volts2) {
          if (volts3 > volts) {
            // Value order is 312
            angle_deg = 270;
          }
          else {
            // Value order is 132
            angle_deg = 330;
          }
        }
        else {
          // Value order is 123
          angle_deg = 30;
        }
      }
      else {
        if (volts2 > volts3) {
          if (volts > volts3) {
            // Value order is 213
            angle_deg = 90;
          }
          else {
            // Value order is 231
            angle_deg = 150;
          }
        } else {
          // Value order is 321
          angle_deg = 210;
        }
      }
    };

    if (shouldPrint) {
      Serial.print("Mic 1: "); Serial.print(volts); Serial.print("\t");
      Serial.print("Mic 2: "); Serial.print(volts2); Serial.print("\t");
      Serial.print("Mic 3: "); Serial.print(volts3); Serial.println("\t");
      Serial.print("Sound detected: "); Serial.println(angle_deg);
    }

    // TODO: Need to fix this function to "simulate" two behaviors in parallel
    if (angle_deg > 0) {
      rightMotor->setSpeed(dspeed);
      leftMotor->setSpeed(dspeed);
      rotateCommand(angle_deg - 180); // delay() goes in here
      stopCommand();
      rightMotor->setSpeed(dspeed);
      leftMotor->setSpeed(dspeed);
      leftMotor->run(FORWARD);
      rightMotor->run(FORWARD);
      pullLeaves(1300);
      stopCommand();
      angle_deg = 0;
    }
    startMillis = 0; // Flag 0 to reset everything
  }
}


//LIGHT SENSING
void senseLight(bool shouldPrint) {
  float frontVal = analogRead(front) + frontCal;
  float leftVal = analogRead(left) + leftCal;
  float rightVal = analogRead(right) + rightCal; 
  y = frontVal - (leftVal + rightVal)/2;
  x = leftVal - rightVal;
  // positive y = forward, positive x = left
  if (abs(x) > threshold_light or abs(y) > threshold_light) {
    // Scale down x and y
    xmult = abs(x) / 100.0;
    ymult = abs(y) / 50.0;

    // xmult is from 0 to 1
    if (xmult > 1) {
      xmult = 1;
    }

    // ymult is from 0.2 to 1
    ymult += forwardbuffer;
    if (ymult > 1) {
      ymult = 1;
    }

    // speeds are from 0 to dspeed
    int turnspeed = dspeed * ymult * (1 - xmult); // turnspeed <= forwardspeed
    int forwardspeed = dspeed * ymult;

    runCommand(forwardspeed, turnspeed);
  } else {
    stopCommand();
  }
  if (shouldPrint) {
    Serial.print(frontVal);
    Serial.print("\t");
    Serial.print(leftVal);
    Serial.print("\t");
    Serial.println(rightVal);
  }
}


// Helper function to set the wheels' speeds
void runCommand(int forwardspeed, int turnspeed) {
  if (x > 0) {
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


// Helper function to set the wheels' speeds
void rotateCommand(int degree) {
  Serial.println(degree);
  leftMotor->run(degree > 0 ? FORWARD : BACKWARD);
  rightMotor->run(degree > 0 ? BACKWARD : FORWARD);
  delay(abs(degree) * 700.0 / 180); // 700 ms rotates 180 degrees
}


// Helper function to stop both wheels
void stopCommand() {
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}
