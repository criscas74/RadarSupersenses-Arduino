#include <Timer.h>
#include <Servo.h>
#include <NewPing.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

const int ADAPIN = 6;
const int TRIGGER_PIN = 12;
const int ECHO_PIN = 11;  // Arduino pin tied to echo pin on the ultrasonic sensor.
const int SERVO_PIN = 9;

const int NUMPIXELS = 24;

const float MAX_DISTANCE = 50; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

const int RADAR_LEDS = 20; //20;
const int RADIUS = 180;

const int STEP_DEGREES = RADIUS / RADAR_LEDS;

const int READINGFREQ = 70;

const int BLINKFREQ = 50;

const float MAX_PERC_DIFF_TO_SPLIT = 0.30;
 
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, ADAPIN, NEO_GRB + NEO_KHZ800);
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
Servo myservo;  // create servo object to control a servo 

Timer t;

float tmpVals[RADAR_LEDS];
int ledVals[RADAR_LEDS];

int currPos = 0;
int adder = 1;
int arrayIndex = 0;

int repeatReading = 0;

int BLINKLEDS[] = {0,NUMPIXELS-1,1000};
boolean blinkLedsStatus[] = {false,false,false};
int blinkColor[] = {pixels.Color(255,255,50),pixels.Color(255,255,50),pixels.Color(255,80,80)};

void setup() 
{ 
  pixels.begin(); // This initializes the NeoPixel library.
  myservo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object 
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.

  t.every(READINGFREQ,takeReading);
  t.every(BLINKFREQ,makeLedBlink);
}
 
void loop() 
{ 
  t.update();
} 

void takeReading()
{
      if (currPos == -1) {
        currPos += 1;
        adder = 1;
        computeAndPrintOut();
      }
      if (currPos == RADAR_LEDS){
        currPos -= 1;
        adder = -1;
        computeAndPrintOut();
      }
      myservo.write(currPos * STEP_DEGREES);
      int sonarReading = sonar.convert_cm(sonar.ping_median(3));
      if(sonarReading == 0) { sonarReading = MAX_DISTANCE; }
      int arrayIndex = currPos;
      if (adder == 1){ arrayIndex -= 2; }
      tmpVals[arrayIndex] = float(MAX_DISTANCE - sonarReading) / float(MAX_DISTANCE);
      currPos  = currPos + adder;  
}

void computeAndPrintOut()
{
      cleanVals();
      splitVals();
      //smoothVals();
      setLedVals();
      checkIfAttacked(); 
}

void cleanVals(){
    for(int i = 1; i < RADAR_LEDS - 1; i += 1)
    {
      if (tmpVals[i] != 0 and tmpVals[i-1] == 0 and tmpVals[i+1] == 0) {
        tmpVals[i] = 0;
        }
    }
}

void splitVals()
{
    blinkLedsStatus[2] = false;
    BLINKLEDS[2] = 1000;
    for(int i = 1; i < RADAR_LEDS - 1; i += 1)
    {
      float difference = 0;
      if (tmpVals[i] != 0 and tmpVals[i-1] != 0){
        difference = abs((tmpVals[i] - tmpVals[i-1]) / tmpVals[i]);
        if (difference > MAX_PERC_DIFF_TO_SPLIT ) { 
          tmpVals[i] = 0;
          blinkLedsStatus[2] = true;
          BLINKLEDS[2] = getLedId(i);
        }          
      }
    }
}

void smoothVals()
{
    for(int i = 0; i < RADAR_LEDS; i += 1)
    {
      tmpVals[i] = float((tmpVals[max(i-1,0)] + tmpVals[i] + tmpVals[min(i+1,RADAR_LEDS-1)])) / 3.00;  
    }
}

void setLedVals()
{
    for(int i = 0; i < RADAR_LEDS; i += 1)
    { 
      int intensity = 0;    
      if (tmpVals[i] > 0) { intensity = tmpVals[i] * 50.0;      }//map(currSmoothedVal,0,1,0,220); }
      ledVals[i] = intensity;
      pixels.setPixelColor(getLedId(i), pixels.Color(intensity,intensity,intensity));  
    }
    pixels.show();
}

void checkIfAttacked()
{
  blinkLedsStatus[0] = blinkLedsStatus[1] = 0;
  if(tmpVals[0] > 0){ blinkLedsStatus[1] = true;}
  if(tmpVals[RADAR_LEDS-1] > 0){ blinkLedsStatus[0] = true;}    
}

void makeLedBlink() {
  for(int i = 0; i < 3; i +=1)
  {
    int currPixelColor = pixels.getPixelColor(BLINKLEDS[i]);
    pixels.setPixelColor(BLINKLEDS[i],0);
    if (blinkLedsStatus[i] == true and currPixelColor == 0){ pixels.setPixelColor(BLINKLEDS[i],blinkColor[i]); }
    pixels.show();
  }
}

int getLedId(int inArrayPointer) {
  return RADAR_LEDS - ((RADAR_LEDS - NUMPIXELS) / 2) - inArrayPointer - 1;
}
