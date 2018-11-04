/****************************************
Example Sound Level Sketch for the 
Adafruit Microphone Amplifier

This code prioritized Sound. 
If the robot gets a sound input, a flag will be set to "TRUE" and it will skip the light sensing for a little while.
****************************************/
// Mic 1 @ 120 degrees connects to A1
// Mic 2 @ 0 degrees connects to A2
// Mic 3 @ 240 degrees connects to A0

// MOTOR STUFF
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
int threshold_sound = 200;
unsigned long moveMillis = millis(); // Starts counting time for how long an angle has been given


//LIGHT STUFF
const int front = A3;
const int left = A4;
const int right = A5;
int x = 0;
int y = 0;
float xmult = 0;
float ymult = 0;

const int threshold = 10; // Lighting threshold
const int dspeed = 40;// default speed
const int xcal = -44; // Need to calibrate xcal and ycal under the normal lighting condition
const int ycal = -38; 
const float forwardbuffer = .2;

void setup() 
{
  
  // MOTOR SHIELD SETUP
  AFMS.begin();
  leftMotor->setSpeed(dspeed);
  rightMotor->setSpeed(dspeed);
  Serial.begin(9600);

  // SOUND PINS SETUP
  //Sets up pins that will be used.
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);

  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(13, HIGH);
  angle_deg = 0;

   
}


void loop() 
{
  // SOUND SENSING
   unsigned long startMillis= millis();  // Start of sample window
   unsigned int peakToPeak = 0;   // peak-to-peak level
   unsigned int peakToPeak2 = 0; // Mic 2
   unsigned int peakToPeak3 = 0; // Mic 3

   unsigned int signalMax = 0; // Mic 1
   unsigned int signalMin = 1024;
   unsigned int signalMax2 = 0; // Mic 2
   unsigned int signalMin2 = 1024;
   unsigned int signalMax3 = 0; // Mic 3
   unsigned int signalMin3 = 1024;


   // collect data for 50 mS
   while (millis() - startMillis < sampleWindow)
   {
      
      sample = analogRead(A1);
      sample2 = analogRead(A2);
      sample3 = analogRead(A0);
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
   }
   peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
   double volts = (peakToPeak * 5.0) / 1024;  // convert to volts
   // Mic 2
   peakToPeak2 = signalMax2 - signalMin2;  // max - min = peak-peak amplitude
   double volts2 = (peakToPeak2 * 5.0) / 1024;  // convert to volts
   // Mic 3
   peakToPeak3 = signalMax3 - signalMin3;  // max - min = peak-peak amplitude
   double volts3 = (peakToPeak3 * 5.0) / 1024;  // convert to volts

//   Serial.println(volts*100);
   Serial.println("Mic 1: "); Serial.print(volts*1000); Serial.print("  ");
   Serial.print("Mic 2: "); Serial.print(volts2*1500); Serial.print("  ");
   Serial.print("Mic 3: "); Serial.print(volts3*1000); Serial.print("  ");

// Maybe later, change to if(volts*1000+volts2*1500+volts3*1000 > threshold*3)
   if(volts*1000 > threshold_sound || volts2*1500>threshold_sound || volts3*1000>threshold_sound){
    if(volts*1000> volts2*1500){
      if(volts3*1000>volts2*1500){
        if(volts3*1000>volts*1000){
          // Value order is 312
          angle_deg = 340;
        }
        else{
          // Value order is 132
          angle_deg = 200;
        }
      }
      else{
        // Value order is 123
        angle_deg = 100;
      }
    }
    else{
      if(volts3*1000>volts2*1500){
        
      }
      else{
        if(volts3*1000>volts2*1500){
          // Value order is 321
          angle_deg = 260;
        }
        else{
          if(volts3*1000>volts*1000){
            // Value order is 231
            angle_deg = 220;
          }
          else{
            // Value order is 213
            angle_deg = 140;
          }
        }
      }
    }
   };
   
   if(angle_deg > 0){
    rightMotor->setSpeed(50);
    leftMotor->setSpeed(50);
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);
    delay(angle_deg*3.333);
    stopCommand();
    leftMotor->run(FORWARD);
    rightMotor->run(FORWARD);
    delay(1000);
    stopCommand();
    angle_deg = 0;
   }
   
  //LIGHT SENSING
  y = analogRead(front) - (analogRead(left) + analogRead(right))/2 + ycal;
    x = analogRead(left) - analogRead(right) + xcal;
    // positive y = forward, positive x = left
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
    
      runCommand(forwardspeed, turnspeed);
    } else {
      stopCommand();
    }
    //printValues();
  
  

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
    Serial.print(xmult);
    Serial.print("\t");
    Serial.println(ymult);
  }
