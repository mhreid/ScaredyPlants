/****************************************
  This code prioritizes Sound.
  If the robot gets a sound input, a flag will
  be set to "TRUE" and it will skip the light
  sensing for a little while.
****************************************/
// Mic 1 @ 120 degrees connects to A9 (FRONT)
// Mic 2 @ 0 degrees connects to A13 (RIGHT)
// Mic 3 @ 240 degrees connects to A11 (LEFT)

// MOTOR STUFFf
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

//includes the motorshield required libraries
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
//Creates the motorshield object
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);


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
const int front = 13;
const int right = 15;
const int left = 11;
int x = 0;
int y = 0;
float xmult = 0;
float ymult = 0;

const int threshold_light = 10;
const int dspeed = 150;// default speed for wheels
const int frontCal = -42; // Need to calibrate xcal and ycal under the normal lighting condition, make it so all == 300 in normal light
const int leftCal = 4;
const int rightCal = 29;
const float forwardbuffer = .2;

//LEAF SERVO STUFF
#include <Servo.h>
int uppos = 0; 
int downpos = 140;
int pos = uppos;  // 0 is open leaves, 140 is closed leaves
bool goingup = false;
Servo leaves1;  // create servo object to control a servo
Servo leaves2;
Servo leaves3;
int leavespos1 = pos; 
int leavespos2 = pos;
int leavespos3 = pos;

void setup()
{
  // MOTOR SHIELD SETUP
  AFMS.begin();
  leftMotor->setSpeed(dspeed);
  rightMotor->setSpeed(dspeed);
  Serial.begin(9600);
  angle_deg = 0;

  //SERVO ATTACH
  leaves1.attach(10);
  leaves2.attach(11);
  leaves3.attach(12);
}


void loop()
{
  senseSound(false);
  senseLight(false);
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
    sample = analogRead(7);
    sample2 = analogRead(9);
    sample3 = analogRead(5);
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

    if (volts > threshold_sound || volts2 > threshold_sound || volts3 > threshold_sound) {
      if (volts > volts2) {
        if (volts3 > volts2) {
          if (volts3 > volts) {
            angle_deg = 270;
          }
          else {
            angle_deg = 330;
          }
        }
        else {
          angle_deg = 30;
        }
      }
      else {
        if (volts2 > volts3) {
          if (volts > volts3) {
            angle_deg = 90;
          }
          else {
            angle_deg = 150;
          }
        } else {
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
      rotateandrun(dspeed, angle_deg);
    }
    startMillis = 0; // Flag 0 to reset everything
  }
}

void rotateandrun(int motor_speed, int angle) {
  rightMotor->setSpeed(motor_speed);
  leftMotor->setSpeed(motor_speed);
  rotateCommand(angle - 180); // delay() goes in here
  stopCommand();
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  int timecheck2 = millis();
  int counter2 = 1;
  while (millis() - timecheck2 < 3000) {
    if (millis() > (3000 / 140)*counter2) {
      pos -= 1;
      leaves1.write(pos); //0 means it is all the way up
      leaves2.write(pos); 
      leaves3.write(pos); 
      counter2 += 1;
    }
  }
  //pullLeaves(false);
  stopCommand();
  angle_deg = 0;
}


// LEAF SHRINKING
// TODO: Get rid of it and integrate the leaf movements into other actions.
void pullLeaves(bool up) {
   if(pos <= downpos && up == false){
    pos +=1;
    leaves1.write(pos);
    leaves2.write(pos);
//    leaves3.write(pos);   // tell servo to go to position in variable 'pos'
    delay(3);
  }
  if(pos >= uppos && up == true ){
    pos -= 1;
    leaves1.write(pos);
    leaves2.write(pos);
//    leaves3.write(pos); // tell servo to go to position in variable 'pos'
    delay(3);
  }
}

//LIGHT SENSING
void senseLight(bool shouldPrint) {
  float frontVal = analogRead(front) + frontCal;
  float leftVal = analogRead(left) + leftCal;
  float rightVal = analogRead(right) + rightCal;
  y = frontVal - (leftVal + rightVal) / 2;
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
    //This makes speed exponential based on proximity
    ymult *= ymult;
    Serial.println(ymult);
    if(ymult > .5){
      pullLeaves(true);
    }
     else{
      pullLeaves(false);
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
  //leftMotor->run(y > 0 ? FORWARD : BACKWARD); // Conditional tenary operator
  //rightMotor->run(y > 0 ? FORWARD : BACKWARD);
  if(y < 0){
    rightMotor->run(FORWARD);
    leftMotor->run(FORWARD);
  }else{
    rightMotor->run(BACKWARD);
    leftMotor->run(BACKWARD);
  }
}


// Helper function to set the wheels' speeds
void rotateCommand(int degree) {
  Serial.println(degree);
  int timecheck = millis();
  int counter1 = 1;
  leftMotor->run(degree > 0 ? FORWARD : BACKWARD);
  rightMotor->run(degree > 0 ? BACKWARD : FORWARD);
  while (millis() - timecheck < abs(degree) * 700.0 / 180) {
    if (millis() > abs(degree)*counter1 * 700.0 / (180 * 140)) {
      pos += 1;
      leaves1.write(pos); //0 means it is all the way up
      leaves2.write(pos); //0 means it is all the way up
      leaves3.write(pos); //0 means it is all the way up
      counter1 += 1;
    }
  }

  //delay(abs(degree) * 700.0 / 180); // 700 ms rotates 180 degrees
}


// Helper function to stop both wheels
void stopCommand() {
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}
