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
int uppos = 10;
int downpos = 160;
int pos = uppos;  // 10 is open leaves, 160 is closed leaves
bool pullingLeaves = false;
Servo leaves1;  // create servo object to control a servo
Servo leaves2;
Servo leaves3;
Servo leaves4;

void setup() {
  // MOTOR SHIELD SETUP
  AFMS.begin();

  // SERIAL SETUP
  // Bluetooth is Serial1 for TX18 RX19
  Serial1.begin(9600);
  Serial.begin(9600);

  //SERVO ATTACH
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  leaves1.attach(2);
  leaves2.attach(3);
  leaves3.attach(4);
  leaves4.attach(5);
  leaves1.write(uppos);
  leaves2.write(uppos);
  leaves3.write(uppos);
  leaves4.write(uppos);
  delay(500);
}

void loop() {
  senseSound(true);
  //    senseLight3(false);
  pullLeaves();
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

  //  Serial1.println(millis() - startMillis);
  // TODO: Need to fix this function to "simulate" two behaviors in parallel
  if (angle_deg > 0) {
    rotateAndRun(110);
    if (angle_deg == 0) {
      startMillis = 0; // Flag 0 to reset everything
    }
    return;
  }

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
      signalMax = sampleBack < signalMax ? signalMax : sampleBack;
      signalMin = sampleBack > signalMin ? signalMin : sampleBack;
    }
    // Mic 2
    if (sampleLeft < 1024)  // toss out spurious readings
    {
      signalMax2 = sampleLeft < signalMax2 ? signalMax2 : sampleLeft;
      signalMin2 = sampleLeft > signalMin2 ? signalMin2 : sampleLeft;
    }
    // Mic 3
    if (sampleRight < 1024)  // toss out spurious readings
    {
      signalMax3 = sampleRight < signalMax3 ? signalMax3 : sampleRight;
      signalMin3 = sampleRight > signalMin3 ? signalMin3 : sampleRight;
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
            angle_deg = 30; // B > R > L
          }
        }
        else {
          angle_deg = 330; // B > L > R
        }
      }
      else {
        if (voltsLeft > voltsRight) {
          if (voltsBack > voltsRight) {
            angle_deg = 270; // L > B > R
          }
          else {
            angle_deg = 210; // L > R > B
          }
        } else {
          angle_deg = 150; // R > L > B
        }
      }
      pullingLeaves = true; // Trigger the servos to pull leaves
    } else {
      startMillis = 0; // Flag 0 to reset everything
    }
    if (shouldPrint) {
      Serial1.print("Mic Back: "); Serial1.print(signalMax - signalMin); Serial1.print("\t");
      Serial1.print("Mic Left: "); Serial1.print(signalMax2 - signalMin2); Serial1.print("\t");
      Serial1.print("Mic Right: "); Serial1.print(signalMax3 - signalMin3); Serial1.println("\t");
      if (angle_deg > 0) {
        Serial1.print("Sound detected: "); Serial1.println(angle_deg);
      }
    }
  }
}

void rotateAndRun(int motor_speed) {
  static unsigned long startMillis;
  int degree = angle_deg - 180;
  if (startMillis == 0) { // At start
    startMillis = millis();
    rightMotor->setSpeed(motor_speed);
    leftMotor->setSpeed(motor_speed);
  } else if (millis() - startMillis < abs(degree) * 1300.0 / 180) { // Rotate: at speed 110, 1300 ms rotates ~180 degrees
    leftMotor->run(degree > 0 ? FORWARD : BACKWARD);
    rightMotor->run(degree > 0 ? BACKWARD : FORWARD);
  } else if (millis() - startMillis < abs(degree) * 1300.0 / 180 + 3000) { // Move forward
    leftMotor->run(BACKWARD);
    rightMotor->run(BACKWARD);
  } else { // Stop
    stopCommand();
    angle_deg = 0;
    startMillis = 0;
  }
}

// LEAF SHRINKING
void pullLeaves() {
  static unsigned long startMillis;
  static int DELAY = 20; // in millis
  int halfTime = abs(downpos - uppos) * DELAY;

  if (pullingLeaves) {
    Serial.print(halfTime);
    Serial.print("\t");
    Serial.println(millis() - startMillis);
    if (startMillis == 0) {
      startMillis = millis();
      pos = uppos;
    } else if (millis() - startMillis < halfTime) { // Pull down
      if (millis() - startMillis > abs(pos - uppos) * DELAY) {
        pos++;
        rotateServos();
      }
    } else if (millis() - startMillis < 2 * halfTime) { // Pull up
      if (millis() - startMillis > halfTime + abs(downpos - pos) * DELAY) {
        pos--;
        rotateServos();
      }
    } else { // Finish the action
      startMillis = 0;
      pullingLeaves = false;
    }
  } else {
    startMillis = 0;
  }
}

void rotateServos() {
  Serial.print("pos: ");
  Serial.println(pos);
  if (pos >= uppos && pos <= downpos) { // Safe check
    leaves1.write(pos);
    leaves2.write(pos);
    leaves3.write(pos);
    leaves4.write(pos);
  }
}

int sign(float x) {
  return x < 0 ? -1 : 1; // Useful function to find sign of x
}

// LIGHT SENSING
// Code for 3 sensors
void senseLight3(bool shouldPrint) {
  if (angle_deg == 0) {
    float y = b->val() - 0.5 * (fr->val() + fl->val());
    float x = sqrt(3) / 2 * (fl->val() - fr->val());
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
}

// Code for 6 sensors
void senseLight6(bool shouldPrint) {
  if (angle_deg == 0) {
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
