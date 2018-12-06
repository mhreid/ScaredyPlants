#include <Servo.h>

Servo myservo;
const int SERVO_PIN = 9;

int current_pos = 0;    // variable to store the servo current's position

void setup() {
  myservo.attach(SERVO_PIN);
  servo_write(10, 50);
}

void loop() {
  int pos = 10;
  while (pos <= 150) {
    pos += 1;
    myservo.write(pos);
    delay(20);
  }
  delay(2000);
  while (pos >= 10) {
    pos -= 1;
    myservo.write(pos);
    delay(20);
  }
  delay(2000);
}

void servo_write(int value, int time_delay) {
  myservo.write(value);
  current_pos = value;
  delay(time_delay);
}

void servo_rotate(int degree, int time_delay) {
  int desired_pos = degree - current_pos;
  if (desired_pos < 0) {
    desired_pos = 0;
  } else if (desired_pos > 180) {
    desired_pos = 180;
  }
  int i = degree > 0 ? +1 : -1;
  while (desired_pos != current_pos) {
    servo_write(current_pos + i, time_delay);
  }
}
