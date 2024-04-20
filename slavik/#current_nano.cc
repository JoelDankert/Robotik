//nano
#include <Wire.h>
#include "Adafruit_TCS34725.h"

// Define the color sensor with appropriate integration time and gain
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

// Define pin numbers for outputs and reset input
const int redPin = 2;
const int blackPin = 3;
const int greenPin = 10;
const int resetPin = 4;
const int LEDpin = 5;
float avgclear = -1;
const int clearF = 300;
const int clearFF = 50;
const int fastchange = 3000;
int startmillis = 0;

const int tickpin = 8;
const int resetmega = 9;
bool firstTickDetected = true; // Flag to check if the first tick has been detected



void setup() {
  Serial.begin(9600);
  startmillis = millis();
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);  // Halt if sensor not found
  }
  tcs.init();

  pinMode(redPin, OUTPUT);
  pinMode(blackPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(LEDpin, OUTPUT);
  pinMode(tickpin, INPUT);
  pinMode(resetmega, OUTPUT);
  pinMode(resetPin, INPUT_PULLUP);  // Enable internal pull-up resistor

  digitalWrite(redPin, LOW);
  digitalWrite(blackPin, LOW);
  digitalWrite(LEDpin, HIGH);
  digitalWrite(resetmega, HIGH);
}

void loop() {
  static int lastTickState = -1; // Stores the last tick state, initialized to -1
  static int unchangedTicks = -150; // Counter for the number of iterations the tick has remained unchanged
  
   bool isRed, isBlack, isGreen;
  detectColor(isRed,isBlack,isGreen);


    if (isRed) {
      Serial.println("quag");
      digitalWrite(redPin, HIGH);
    }

    if (isBlack) {
      Serial.println("bobr");
      digitalWrite(blackPin, HIGH);
    }
    if (isGreen) {
      Serial.println("trier");
      digitalWrite(greenPin, HIGH);
    }
      Serial.println(" ");
  
  
  if (digitalRead(resetPin) == LOW) {
    digitalWrite(redPin, LOW);
    digitalWrite(blackPin, LOW);
    digitalWrite(greenPin, LOW);
  }

  int currentTickState = digitalRead(tickpin);
      Serial.println(unchangedTicks);
 if (!firstTickDetected) {
    if (currentTickState != lastTickState) {
      firstTickDetected = true; // Set flag to true once the first change is detected
      Serial.println("First tick detected. Monitoring for stability.");
    }
  } else {
    // Continue with tick monitoring only after the first tick has been detected
    if (currentTickState == lastTickState) {
      // If the tick state hasn't changed
      unchangedTicks++;
      lastTickState = currentTickState;
    } else {
      unchangedTicks = 0;
      lastTickState = currentTickState;
    }
    if (unchangedTicks >= 80) {
      Serial.println("Tick signal stopped. Resetting Mega...");
      digitalWrite(resetmega, LOW); // Assuming LOW triggers a reset
      delay(100); // Hold the reset for 100ms
      digitalWrite(resetmega, HIGH);
      unchangedTicks = -150; // Reset the counter after resetting
      delay(2000); // Hold the reset for 100ms
      firstTickDetected = false;
      lastTickState = digitalRead(tickpin);
    }
  
  }
  delay(10);
}


void detectColor(bool &redAmount, bool &blackAmount, bool &greenAmount) {
  delay(10);
  uint16_t clear, red, green, blue;
  tcs.getRawData(&red, &green, &blue, &clear);

  // Normalize the values
  float r = red / (float)clear;
  float g = green / (float)clear;
  float b = blue / (float)clear;

  // Define thresholds for red, white, and black detection
  const float redThreshold = 1.2;
  const float greenThreshold = 1.5;
  const float blackThreshold = 0.3;   
  const float maxblackdiff = 0.5;

  if (avgclear == -1){avgclear = clear;}
  if(millis()>startmillis+fastchange){
    avgclear = (avgclear * clearF + clear) / (clearF+1);
  }
  else{
    avgclear = (avgclear * clearFF + clear) / (clearFF+1);
  }


  if (avgclear < 1){
    avgclear = 1;
  }


  // Reset amounts
  redAmount = false;
  blackAmount = false;
  greenAmount = false;

  // Detect colors based on thresholds
  if (r > g*redThreshold && r > b*redThreshold && clear > 8) {
    redAmount = true; // Red detected
  }
  if (g > b*greenThreshold && r > b*greenThreshold && clear > 8) {
    greenAmount = true; // green detected
  }

  float maxDiff = max(max(abs(r - g), abs(r - b)), abs(g - b));

  if (clear / avgclear < blackThreshold && maxDiff <= maxblackdiff) {
    blackAmount = true; // black detected
  }
  Serial.println(" ");
  Serial.println(r);
  Serial.println(g);
  Serial.println(b);
  Serial.println(clear);
  Serial.println(avgclear);

}
