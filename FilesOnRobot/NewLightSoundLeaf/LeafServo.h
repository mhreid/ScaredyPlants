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