/****************************************
Example Sound Level Sketch for the 
Adafruit Microphone Amplifier
****************************************/
// Mic 1 connects to A0
// Mic 2 connects to A1
// Mic 3 connects to A2
// If we ever get a Mic 4, it will connect to A3
// If we ever use an IMU, it will connect to A4 and A5


const int sampleWindow = 1000; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;
unsigned int sample2;
unsigned int sample3;

void setup() 
{
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
  
   Serial.begin(9600);
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
      sample = analogRead(A0);
      sample2 = analogRead(A1);
      sample3 = analogRead(A2);
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

   //Serial.println(volts*100);
   Serial.println("Mic 1: "); Serial.print(volts*1000); Serial.print("  ");
   Serial.print("Mic 2: "); Serial.print(volts2*1500); Serial.print("  ");
   Serial.print("Mic 3: "); Serial.print(volts3*1000); Serial.print("  ");
}
