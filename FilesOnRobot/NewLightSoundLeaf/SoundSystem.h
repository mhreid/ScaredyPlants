// SOUND STUFF
// Sets up variables that will be used
// Mic 1 @ 180 degrees connects to A9 (BACK)
// Mic 2 @ 30 degrees connects to A7 (RIGHT)
// Mic 3 @ 330 degrees connects to A8 (LEFT)
const int sampleWindow = 100; // Sample window width in mS (50 mS = 20Hz)
unsigned int sampleFront;
unsigned int sampleRight;
unsigned int sampleLeft;
int angle_deg = 0;
int threshold_sound = 100;
unsigned long moveMillis = millis(); // Starts counting time for how long an angle has been given
const int frontSound = A9;
const int rightSound = A7;
const int leftSound = A8;
