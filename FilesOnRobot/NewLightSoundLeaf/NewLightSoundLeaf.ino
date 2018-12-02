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
unsigned int sampleBack;
unsigned int sampleRight;
unsigned int sampleLeft;
unsigned int angle_deg = 0;
int threshold_sound = 50;
unsigned long moveMillis = millis(); // Starts counting time for how long an angle has been given
const int backSound = 2;
const int rightSound = 8;
const int leftSound = 0;


//LIGHT STUFF
const int fLight = A15;
const int frLight = A14;
const int flLight = A10;
const int bLight = A12;
const int brLight = A13;
const int blLight = A11;
float x = 0;
float y = 0;
float xmult = 0;
float ymult = 0;
class LightSensor {
  public:
    int min = 1024;
    int max = 0;
    int pin;
    LightSensor(int pin) {
      this->pin = pin;
    }
    int mean() {
      return (min + max) / 2;
    }
    float val() {
      float pinVal = analogRead(pin); // Convert to float
      return pinVal * (max - min) / max - min + 500; // Scale it, then shift it to 500
    }
    int updateMinMax() {
      int value = analogRead(pin);
      if (value < this->min) this->min = value;
      if (value > this->max) this->max = value;
    }
};

const int threshold_light = 10;
const int dspeed = 150;// default speed for wheels
const LightSensor *f = new LightSensor(fLight); // Need to calibrate xcal and ycal under the normal lighting condition, make it so all == 800 in normal light
const LightSensor *fr = new LightSensor(frLight);
const LightSensor *fl = new LightSensor(flLight);
const LightSensor *b = new LightSensor(bLight);
const LightSensor *br = new LightSensor(brLight);
const LightSensor *bl = new LightSensor(blLight);
//const int fCal = -22 - 17; // Need to calibrate xcal and ycal under the normal lighting condition, make it so all == 800 in normal light
//const int frCal = -56 + 34;
//const int flCal = 54 - 54;
//const int bCal = -29 - 14;
//const int brCal = -92 + 79;
//const int blCal = 12 - 46;
//const int fCal = -22; // Need to calibrate xcal and ycal under the normal lighting condition, make it so all == 800 in normal light
//const int frCal = -56;
//const int flCal = 54;
//const int bCal = -29;
//const int brCal = -92;
//const int blCal = 12;
const float forwardbuffer = .2;


//LEAF SERVO STUFF
#include <Servo.h>
int uppos = 0;
int downpos = 140;
int pos = uppos;  // 0 is open leaves, 140 is closed leaves
bool up = false;
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

  // LIGHT CALIBRATION
  calibrateLight();
}

void loop()
{

  //  senseSound(false);
  senseLight(true);
}


// SOUND SENSING
/**
   Initially rotate 360 degrees to calibrate the offsets
   of the light sensors. Each iteration will update the
   max and the min values of the light sensors.
*/
void calibrateLight() {
  // Set motor speeds to appropriate values
  leftMotor->setSpeed(70);
  rightMotor->setSpeed(70);
  leftMotor->run(FORWARD);
  rightMotor->run(BACKWARD);

  // Rotate the robot
  // TODO: find appropriate stop time
  long startMillis = millis();
  while (millis() - startMillis < 360 * 500) {
    f->updateMinMax();
    fr->updateMinMax();
    fl->updateMinMax();
    b->updateMinMax();
    br->updateMinMax();
    bl->updateMinMax();
  }

  // Re-set motor speeds
  leftMotor->setSpeed(dspeed);
  rightMotor->setSpeed(dspeed);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}

void senseSound(bool shouldPrint) {
  static unsigned long startMillis;  // Start of sample window
  int peakToPeak = 0;   // peak-to-peak level
  int peakToPeak2 = 0; // Mic 2
  int peakToPeak3 = 0; // Mic 3

  static int signalMax; // Mic 1
  static int signalMin;
  static int signalMax2; // Mic 2
  static int signalMin2;
  static int signalMax3; // Mic 3
  static int signalMin3;


  if (startMillis == 0) { // Re-initialize everything
    startMillis = millis();
    signalMax = 0;
    signalMax2 = 0;
    signalMax3 = 0;
    signalMin = 1024;
    signalMin2 = 1024;
    signalMin3 = 1024;
    angle_deg = 0;
  }
  // collect data for 50 mS
  if (millis() - startMillis < sampleWindow)
  {
    sampleBack = analogRead(backSound);
    sampleLeft = analogRead(leftSound);
    sampleRight = analogRead(rightSound);
    // Mic 1
    if (sampleBack < 1024)  // toss out spurious readings
    {
      if (sampleBack > signalMax)
      {
        signalMax = sampleBack;  // save just the max levels
      }
      else if (sampleBack < signalMin)
      {
        signalMin = sampleBack;  // save just the min levels
      }
    }
    // Mic 2
    if (sampleLeft < 1024)  // toss out spurious readings
    {
      if (sampleLeft > signalMax2)
      {
        signalMax2 = sampleLeft;  // save just the max levels
      }
      else if (sampleLeft < signalMin2)
      {
        signalMin2 = sampleLeft;  // save just the min levels
      }
    }
    // Mic 3
    if (sampleRight < 1024)  // toss out spurious readings
    {
      if (sampleRight > signalMax3)
      {
        signalMax3 = sampleRight;  // save just the max levels
      }
      else if (sampleRight < signalMin3)
      {
        signalMin3 = sampleRight;  // save just the min levels
      }
    }
  } else {
    peakToPeak = abs(signalMax - 32);  // max - min = peak-peak amplitude
    double voltsBack = 1000 * (peakToPeak * 5.0) / 1024;  // convert to volts
    // Mic 2
    peakToPeak2 = abs(signalMax2 - 37);  // max - min = peak-peak amplitude
    double voltsLeft = 1000 * (peakToPeak2 * 5.0) / 1024;  // convert to volts
    // Mic 3
    peakToPeak3 = abs(signalMax3 - 48);  // max - min = peak-peak amplitude
    double voltsRight = 1000 * (peakToPeak3 * 5.0) / 1024;  // convert to volts

    if (voltsBack > threshold_sound || voltsLeft > threshold_sound || voltsRight > threshold_sound) {
      if (voltsBack > voltsLeft) {
        if (voltsRight > voltsLeft) {
          if (voltsRight > voltsBack) {
            angle_deg = 90; // R > B > L
          }
          else {
            angle_deg = 150; // B > R > L
          }
        }
        else {
          angle_deg = 210; // B > L > R
        }
      }
      else {
        if (voltsLeft > voltsRight) {
          if (voltsBack > voltsRight) {
            angle_deg = 270; // L > B > R
          }
          else {
            angle_deg = 330; // L > R > B
          }
        } else {
          angle_deg = 30; // R > L > B
        }
      }
    };

    if (shouldPrint) {
      Serial.print("Mic Back: "); Serial.print(signalMax - 32); Serial.print("\t");
      Serial.print("Mic Left: "); Serial.print(signalMax2 - 37); Serial.print("\t");
      Serial.print("Mic Right: "); Serial.print(signalMax3 - 48); Serial.println("\t");
    }

    // TODO: Need to fix this function to "simulate" two behaviors in parallel
    if (angle_deg > 0) {
      Serial.print("Sound detected: "); Serial.println(angle_deg);
      //      rotateandrun(dspeed, angle_deg);
    }
    startMillis = 0; // Flag 0 to reset everything
  }
}

void rotateandrun(int motor_speed, int angle) {
  rightMotor->setSpeed(motor_speed);
  leftMotor->setSpeed(motor_speed);
  //rotateCommand(angle - 180); // delay() goes in here
  //stopCommand();
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  int timecheck2 = millis();
  int counter2 = 1;
  while (millis() - timecheck2 < 3000) {
    if (millis() > (3000 / 140)*counter2) {
      pos -= 1;
      leaves1.write(pos); //0 means it is all the way up
      counter2 += 1;
    }
  }
  //pullLeaves(false);
  //  stopCommand();
  angle_deg = 0;
}


// LEAF SHRINKING
// TODO: Get rid of it and integrate the leaf movements into other actions.
void pullLeaves(bool up) {
  if (pos <= downpos && up == false) {
    pos += 1;
    leaves1.write(pos);
    leaves2.write(pos);
    leaves3.write(pos);   // tell servo to go to position in variable 'pos'
    delay(3);
  }
  if (pos >= uppos && up == true ) {
    pos -= 1;
    leaves1.write(pos);
    leaves2.write(pos);
    leaves3.write(pos); // tell servo to go to position in variable 'pos'
    delay(3);
  }
}


//LIGHT SENSING
void senseLight(bool shouldPrint) {
  y = f->val() + 0.5 * (fr->val() + fl->val()) - (b->val() + 0.5 * (br->val() + bl->val()));
  x = sqrt(3) / 2 * (fl->val() + bl->val() - fr->val() - br->val());
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
    //Serial.println(ymult);
    if (ymult > .5) {
      pullLeaves(true);
    }
    else {
      pullLeaves(false);
    }

    // speeds are from 0 to dspeed
    int turnspeed = dspeed * ymult * (1 - xmult); // turnspeed <= forwardspeed
    int forwardspeed = dspeed * ymult;
    Serial.print(x);
    Serial.print("\t");
    Serial.println(y);
    runCommand(forwardspeed, turnspeed);
  } else {
    stopCommand();
  }
  if (shouldPrint) {
    Serial.print("f ");
    Serial.print(f->val());
    Serial.print("\t fl ");
    Serial.print(fl->val());
    Serial.print("\t fr ");
    Serial.println(fr->val());

    Serial.print("b ");
    Serial.print(b->val());
    Serial.print("\t bl ");
    Serial.print(bl->val());
    Serial.print("\t br ");
    Serial.println(br->val());

    //    Serial.print(xmult);
    //    Serial.print("\t");
    //    Serial.println(ymult);
  }
}


// Helper function to set the wheels' speeds
void runCommand(int forwardspeed, int turnspeed) {
  if (x > 0) {
    // Turn left
    rightMotor->setSpeed(forwardspeed > 150 ? 150 : forwardspeed);
    leftMotor->setSpeed(turnspeed > 150 ? 150 : turnspeed);
  } else {
    // Turn right
    rightMotor->setSpeed(turnspeed > 150 ? 150 : turnspeed);
    leftMotor->setSpeed(turnspeed > 150 ? 150 : turnspeed);
  }
  //leftMotor->run(y > 0 ? FORWARD : BACKWARD); // Conditional tenary operator
  //rightMotor->run(y > 0 ? FORWARD : BACKWARD);
  if (y < 0) {
    rightMotor->run(FORWARD);
    leftMotor->run(FORWARD);
  } else {
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
