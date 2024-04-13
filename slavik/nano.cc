#include <Wire.h>
#include "Adafruit_TCS34725.h"

// Define color sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Define pin numbers for outputs and reset input
const int redPin = 2;
const int blackPin = 3;
const int resetPin = 4;

void setup() {
  // Initialize serial communication and sensor
  Serial.begin(9600);
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt if sensor not found
  }

  // Initialize the pins
  pinMode(redPin, OUTPUT);
  pinMode(blackPin, OUTPUT);
  pinMode(resetPin, INPUT_PULLUP); // Use internal pull-up resistor

  // Set initial state
  digitalWrite(redPin, LOW);
  digitalWrite(blackPin, LOW);
}

void loop() {
  // Check for reset signal
  if (digitalRead(resetPin) == LOW) {
    digitalWrite(redPin, LOW);
    digitalWrite(blackPin, LOW);
  } else {
    // Detect color and set outputs accordingly
    String color = detectColor();
    if (color == "red") {
      digitalWrite(redPin, HIGH);
    } else if (color == "black") {
      digitalWrite(blackPin, HIGH);
    }
  }

  delay(100); // Small delay to prevent excessive processing
}

String detectColor() {
  uint16_t clear, red, green, blue;
  tcs.getRawData(&red, &green, &blue, &clear);

  if (red < 20 && blue < 20 && green < 20) {
    return "black";
  } else if (green * 1.4 < red && blue * 1.4 < red) {
    return "red";
  } else {
    return "none"; // If none of the conditions are met
  }
}
