/****************************************
  This code prioritizes Sound.
  If the robot gets a sound input, a flag will
  be set to "TRUE" and it will skip the light
  sensing for a little while.
****************************************/
// MOTOR STUFFf
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

#include "LightSystem.h"
#include "SoundSystem.h"

const int dspeed = 70; // default speed for wheels

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

void setup() {
  // MOTOR SHIELD SETUP
  AFMS.begin();

  // SERIAL SETUP
  // Bluetooth is Serial1 for TX18 RX19
  Serial1.begin(9600);

  //SERVO ATTACH
  leaves1.attach(10);
  leaves2.attach(11);
  leaves3.attach(12);

}

void loop() {
  senseSound(true);
  senseLight(false);
}


// SOUND SENSING
void senseSound(bool shouldPrint) {
  static unsigned long startMillis;  // Start of sample window
  static int signalMax; // Mic 1
  static int signalMin;
  static int signalMax2; // Mic 2
  static int signalMin2;
  static int signalMax3; // Mic 3
  static int signalMin3;


  if (startMillis == 0) { // Re-initialize everything
    startMillis = millis();
    signalMax = signalMax2 = signalMax3 = 0;
    signalMin = signalMin2 = signalMin3 = 1024;
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
    int peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
    double voltsBack = 1000 * (peakToPeak * 5.0) / 1024;  // convert to volts
    // Mic 2
    int peakToPeak2 = signalMax2 - signalMin2;  // max - min = peak-peak amplitude
    double voltsLeft = 1000 * (peakToPeak2 * 5.0) / 1024;  // convert to volts
    // Mic 3
    int peakToPeak3 = signalMax3 - signalMin3;  // max - min = peak-peak amplitude
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
      Serial1.print("Mic Back: "); Serial1.print(signalMax - signalMin); Serial1.print("\t");
      Serial1.print("Mic Left: "); Serial1.print(signalMax2 - signalMin2); Serial1.print("\t");
      Serial1.print("Mic Right: "); Serial1.print(signalMax3 - signalMin3); Serial1.println("\t");
    }

    // TODO: Need to fix this function to "simulate" two behaviors in parallel
    if (angle_deg > 0) {
      Serial1.print("Sound detected: "); Serial1.println(angle_deg);
      rotateandrun(110, angle_deg);
    }
    startMillis = 0; // Flag 0 to reset everything
  }
}

void rotateandrun(int motor_speed, int angle) {
  rightMotor->setSpeed(motor_speed);
  leftMotor->setSpeed(motor_speed);
  rotateCommand(angle - 180); // delay() goes in here
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  unsigned long timecheck2 = millis();
  int counter2 = 1;
  while (millis() - timecheck2 < 3000) {
    if (millis() > (3000 / 140)*counter2) {
      pos -= 1;
      leaves1.write(pos); //0 means it is all the way up
      counter2 += 1;
    }
  }
  stopCommand();
  angle_deg = 0;
}

void rotateCommand(int degree) {
  unsigned long timecheck = millis();
  int counter1 = 1;
  rightMotor->run(degree > 0 ? FORWARD : BACKWARD);
  leftMotor->run(degree > 0 ? BACKWARD : FORWARD);
  Serial1.print(millis() - timecheck);
  while (millis() - timecheck < abs(degree) * 1300.0 / 180) { // at speed 110, 1300 ms rotates ~180 degrees
    if (millis() > abs(degree) * counter1 * 1300.0 / (180 * 140)) {
      pos += 1;
      leaves1.write(pos); //0 means it is all the way up
      counter1 += 1;
    }
  }
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

int sign(float x) {
  return x < 0 ? -1 : 1; // Useful function to find sign of x
}

// LIGHT SENSING
void senseLight(bool shouldPrint) {
  float y = b->val() + 0.5 * (br->val() + bl->val()) - (f->val() + 0.5 * (fr->val() + fl->val()));
  float x = sqrt(3) / 2 * (fl->val() + bl->val() - fr->val() - br->val());
  // Scale values if needed
  float xmult = x / 50;
  float ymult = y / 50;
  float vSum = dspeed * ymult; // vR + vL = vSum
  float vDiff = dspeed * xmult * sign(ymult); // vR - vL = sumX
  float vR = (vSum + vDiff) / 2;
  float vL = (vSum - vR);
  if (abs(vL) > threshold_light || abs(vR) > threshold_light) {
    runCommand(vL, vR);
  } else {
    stopCommand();
  }

  if (shouldPrint) {
    Serial1.print(vL);
    Serial1.print("\t");
    Serial1.println(vR);
    Serial1.print("f ");
    Serial1.print(f->val());
    Serial1.print("\t fl ");
    Serial1.print(fl->val());
    Serial1.print("\t fr ");
    Serial1.println(fr->val());

    Serial1.print("b ");
    Serial1.print(b->val());
    Serial1.print("\t bl ");
    Serial1.print(bl->val());
    Serial1.print("\t br ");
    Serial1.println(br->val());
  }
}

// Helper function to set the wheels' speeds
void runCommand(float vL, float vR) {
  leftMotor->setSpeed(abs(vL) > 140 ? 140 : (abs(vL) < dspeed ? dspeed : abs(vL))); // Conditional tenary operator
  rightMotor->setSpeed(abs(vR) > 140 ? 140 : (abs(vR) < dspeed ? dspeed : abs(vR)));
  leftMotor->run(vL > 0 ? FORWARD : BACKWARD);
  rightMotor->run(vR > 0 ? FORWARD : BACKWARD);
}

// Helper function to stop both wheels
void stopCommand() {
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}
