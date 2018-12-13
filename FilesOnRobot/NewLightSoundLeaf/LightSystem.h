// LIGHT STUFF
// Sets up variables that will be used
//const int fLight = A15;
//const int frLight = A14;
//const int flLight = A10;
//const int bLight = A12;
//const int brLight = A13;
//const int blLight = A11;
const int fLight = A15;
const int frLight = A11;
const int flLight = A15;
const int bLight = A13;
const int brLight = A13;
const int blLight = A11;
const int threshold_light = 70;
class LightSensor {
  public:
    int min = 1024;
    int max = 0;
    int pin;
    int calValue;
    LightSensor(int pin) {
      this->pin = pin;
      this->calValue = 0;
    }
    LightSensor(int pin, int calValue) {
      this->pin = pin;
      this->calValue = calValue;
    }
    int mean() {
      return (min + max) / 2;
    }
    float val() {
      return analog() + this->calValue;
    }
    int analog() {
      return analogRead(pin);
    }
    int updateMinMax() {
      int value = analogRead(pin);
      if (value < this->min) this->min = value;
      if (value > this->max) this->max = value;
    }
};
LightSensor *f = new LightSensor(fLight); // Need to calibrate xcal and ycal under the normal lighting condition, make it so all == 800 in normal light
LightSensor *fr = new LightSensor(frLight, -12);
LightSensor *fl = new LightSensor(flLight, 36);
LightSensor *b = new LightSensor(bLight, 22);
LightSensor *br = new LightSensor(brLight);
LightSensor *bl = new LightSensor(blLight);
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
