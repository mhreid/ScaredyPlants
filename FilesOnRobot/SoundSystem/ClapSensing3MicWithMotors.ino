/****************************************
Example Sound Level Sketch for the 
Adafruit Microphone Amplifier
****************************************/
// Mic 1 @ 120 degrees connects to A1
// Mic 2 @ 0 degrees connects to A2
// Mic 3 @ 240 degrees connects to A0
// If we ever get a Mic 4, it will connect to A3
// If we ever use an IMU, it will connect to A4 and A5

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

//includes the motorshield required libraries
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
//Creates the motorshield object
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

// Sets up Variables that will be used
const int sampleWindow = 100; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;
unsigned int sample2;
unsigned int sample3;
unsigned int angle_deg = 0;
int threshold = 200;
unsigned long moveMillis = millis(); // Starts counting time for how long an angle has been given

void setup() 
{

  // Sets up motor shield stuff
  AFMS.begin();
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
  Serial.begin(9600);
  
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
   if(volts*1000 > threshold || volts2*1500>threshold || volts3*1000>threshold){
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
    //Serial.print("Angle Heading: "); Serial.print(angle_deg); Serial.print("  ");
    rightMotor->setSpeed(50);
    leftMotor->setSpeed(50);
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);
    delay(angle_deg*3.333);
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
    angle_deg = 0;
   }
//   Serial.print("Angle Heading: "); Serial.print(angle_deg); Serial.print("  ");
   
}
