#include <Wire.h>
#include "Adafruit_TCS34725.h"

// Define the color sensor with appropriate integration time and gain
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

// Define pin numbers for outputs and reset input
const int redPin = 2;
const int blackPin = 3;
const int resetPin = 4;
const int LEDpin = 5;
float avgclear = -1;
const int clearF = 70;
const int clearFF = 10;
const int fastchange = 3000;
int startmillis = 0;


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
  pinMode(LEDpin, OUTPUT);
  pinMode(resetPin, INPUT_PULLUP);  // Enable internal pull-up resistor

  digitalWrite(redPin, LOW);
  digitalWrite(blackPin, LOW);
}

void loop() {
  if (digitalRead(resetPin) == LOW) {
    digitalWrite(redPin, LOW);
    digitalWrite(blackPin, LOW);
  } else {
    bool isRed, isBlack;
    detectColor(isRed,isBlack);


    if (isRed) {
      Serial.println("quag");
      digitalWrite(redPin, HIGH);
    }

    if (isBlack) {
      Serial.println("bobr");
      digitalWrite(blackPin, HIGH);
    }
      Serial.println(" ");
  }

  delay(100);
}
void detectColor(bool &redAmount, bool &blackAmount) {
  digitalWrite(LEDpin, HIGH);
  delay(10);
  uint16_t clear, red, green, blue;
  tcs.getRawData(&red, &green, &blue, &clear);

  // Normalize the values
  float r = red / (float)clear;
  float g = green / (float)clear;
  float b = blue / (float)clear;

  // Define thresholds for red, white, and black detection
  const float redThreshold = 1.8;   // Example threshold for red
  const float blackThreshold = 0.7;   // Example threshold for red

  if (avgclear == -1){avgclear = clear;}
  if(millis()>startmillis+fastchange){
    avgclear = (avgclear * clearF + clear) / (clearF+1);
  }
  else{
    avgclear = (avgclear * clearFF + clear) / (clearFF+1);
  }


  // Reset amounts
  redAmount = false;
  blackAmount = false;

  // Detect colors based on thresholds
  if (r > g*redThreshold && r > b*redThreshold && clear > 5) {
    redAmount = true; // Red detected
  }
  if (clear/avgclear < blackThreshold) {
    blackAmount = true; // Red detected
  }
  //Serial.println(" ");
  //Serial.println(r);
  //Serial.println(g);
  //Serial.println(b);
  //Serial.println(clear);
  
  digitalWrite(LEDpin, LOW);

}


