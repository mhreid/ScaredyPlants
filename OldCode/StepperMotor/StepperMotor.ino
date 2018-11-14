#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *stepMotor = AFMS.getMotor(2);

boolean shouldPullLeaves = false;

void setup() {
  AFMS.begin();  
  Serial.begin(9600);
  stepMotor->setSpeed(55); // Speed at 55 requires 1350 ms for 1 revolution
  shouldPullLeaves = true;
}

void loop() {
  if (shouldPullLeaves) {
    pullLeaves(1350);
  } else {
    delay(1000);
    shouldPullLeaves = true;
  }

  // Simulate other commands
  delay(10);
}

void pullLeaves(float angle) {
  if (!shouldPullLeaves) {
    return;
  }
  // static variables only exist in this func
  static float accum_time; 
  static float prev_time;
  static float pulling_time; 
  static float break_time;
  static float error_time;
  
  accum_time += prev_time == 0 ? 0 : millis() - prev_time; // Initialize accum_time
  prev_time = millis();
  if (accum_time < angle) {
    stepMotor->run(FORWARD); // Pulling in for 1300 ms
  } else if (accum_time < angle + 1000) {
    pulling_time = pulling_time == 0 ? accum_time : pulling_time; // Initialize pulling_time
    stepMotor->run(RELEASE); // Break for 1000 ms; otherwise the motor would not respond fast enough
  } else {
    break_time = break_time == 0 ? accum_time : break_time; // Initialize break_time
    if (accum_time < break_time + pulling_time + error_time) {
      stepMotor->run(BACKWARD); // Releasing for accum_time ms
    } else {
      stepMotor->run(RELEASE); // Done
      error_time += (break_time + pulling_time) - accum_time;
      accum_time = 0;
      prev_time = 0;
      pulling_time = 0;
      break_time = 0;
      shouldPullLeaves = false;
    }
  }
}
