/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo;  
const int SERVO_PIN = 9;

int current_pos = 0;    // variable to store the servo current's position

void setup() {
  myservo.attach(SERVO_PIN);
  servo_write(0, 50);
  rotate(180, 15);
  rotate(-180, 15);
}

void loop() {

}

void servo_write(int value, int time_delay) {
  myservo.write(value);
  delay(time_delay);
  current_pos = value;
}

void rotate(int degree, int time_delay) {
  int desired_pos = current_pos + degree;
  if (desired_pos < 0) {
    desired_pos = 0;
  } else if (desired_pos > 180) {
    desired_pos = 180;
  }
  int i = degree > 0 ? +1 : -1;
  while (desired_pos != current_pos) {
    servo_write(current_pos + i, 15);
  }
}
