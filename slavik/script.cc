#include <Adafruit_TCS34725.h>
#include <Servo.h>

// Motor control pins
#define motorPinL1 22
#define motorPinL2 24
#define motorPinR1 26
#define motorPinR2 28
#define motorSpeedL 8
#define motorSpeedR 9

// Ultrasonic sensor pins
#define echoPinFront 3
#define trigPinFront 2
#define echoPinLeft 5
#define trigPinLeft 4
#define echoPinRightFront 7
#define trigPinRightFront 6
#define echoPinRightBack 9
#define trigPinRightBack 8

// Global variables for speed control and sensor centerthresh
float globalSpeed = 0.2; // Speed scale from 0 to 1
#define frontSensorOffset 10 // Example value in centimeters
#define mazeGridSize 40 // Example value in centimeters, adjust as per your maze

// Color sensor setup
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

void setup() {
  Serial.begin(9600);
  pinMode(motorPinL1, OUTPUT);
  pinMode(motorPinL2, OUTPUT);
  pinMode(motorPinR1, OUTPUT);
  pinMode(motorPinR2, OUTPUT);
  pinMode(motorSpeedL, OUTPUT);
  pinMode(motorSpeedR, OUTPUT);
  
  // Initialize ultrasonic sensors
  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinRightFront, OUTPUT);
  pinMode(echoPinRightFront, INPUT);
  pinMode(trigPinRightBack, OUTPUT);
  pinMode(echoPinRightBack, INPUT);

  // Initialize color sensor
  if (!tcs.begin()) {
    Serial.println("Couldn't find color sensor");
  }
}

// Function to control motor speed
void setMotorSpeed(float speed) {
  analogWrite(motorSpeedL, speed * 255);
  analogWrite(motorSpeedR, speed * 255);
}

// Functions for movement
void moveForward(int duration) {
  digitalWrite(motorPinL1, LOW);
  digitalWrite(motorPinL2, HIGH);
  digitalWrite(motorPinR1, LOW);
  digitalWrite(motorPinR2, HIGH);
  delay(duration);
}

void moveBackward(int duration) {
  digitalWrite(motorPinL1, HIGH);
  digitalWrite(motorPinL2, LOW);
  digitalWrite(motorPinR1, HIGH);
  digitalWrite(motorPinR2, LOW);
  delay(duration);
}

void turnLeft(int duration) {
  digitalWrite(motorPinL1, HIGH);
  digitalWrite(motorPinL2, LOW);
  digitalWrite(motorPinR1, LOW);
  digitalWrite(motorPinR2, HIGH);
  delay(duration);
}

void turnRight(int duration) {
  digitalWrite(motorPinL1, LOW);
  digitalWrite(motorPinL2, HIGH);
  digitalWrite(motorPinR1, HIGH);
  digitalWrite(motorPinR2, LOW);
  delay(duration);
}

// Function to get sensor distance
int getSensorDistance(int sensorPinTrig, int sensorPinEcho) {
  digitalWrite(sensorPinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(sensorPinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensorPinTrig, LOW);
  long duration = pulseIn(sensorPinEcho, HIGH);
  if (duration == 0) {
    return 1000; // Out of range
  }
  return duration * 0.034 / 2;
}

// Function to detect color
String detectColor() {
  uint16_t clear, red, green, blue;
  tcs.getRawData(&red, &green, &blue, &clear);
  if (red > green && red > blue && red > 100) {
    return "red";
  } else if (red < 50 && green < 50 && blue < 50) {
    return "black";
  } else {
    return "none";
  }
}

// Function to get sensor distance based on specified sensor
int getSensor(String sensor) {
  int sensorPinTrig, sensorPinEcho;
  float conversionvalue = 1;
  
  // Determine which sensor is being requested
  if (sensor == "L") {
    sensorPinTrig = trigPinLeft;
    sensorPinEcho = echoPinLeft;
  } else if (sensor == "F") {
    sensorPinTrig = trigPinFront;
    sensorPinEcho = echoPinFront;
  } else if (sensor == "RF") {
    sensorPinTrig = trigPinRightFront;
    sensorPinEcho = echoPinRightFront;
  } else if (sensor == "RB") {
    sensorPinTrig = trigPinRightBack;
    sensorPinEcho = echoPinRightBack;
  } else {
    // If the sensor code is invalid, return -1 indicating an error
    return -1;
  }

  // Activate the specified sensor and measure the distance
  digitalWrite(sensorPinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(sensorPinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensorPinTrig, LOW);
  long duration = pulseIn(sensorPinEcho, HIGH);

  // Calculate and return the distance
  if (duration == 0) {
    // Return a high value to indicate out of range
    return 1000;
  }
  return (duration * 0.034 / 2)*conversionvalue;
}

// Updated center function to use getSensor with strings
void center() {
  int centerthresh = 1; // centerthresh value for alignment accuracy
  int distanceRightFront = getSensor("RF");
  int distanceRightBack = getSensor("RB");
  
  // Adjust the robot until the sensor values are within the centerthresh
  while (abs(distanceRightFront - distanceRightBack) > centerthresh) {
    if (distanceRightFront > distanceRightBack) {
      // If the front sensor detects a longer distance, turn the robot right slightly
      turnRight(100); // Adjust the duration as needed for slight adjustments
    } else {
      // If the back sensor detects a longer distance, turn the robot left slightly
      turnLeft(100); // Adjust the duration as needed for slight adjustments
    }
    // Update distance measurements
    distanceRightFront = getSensor("RF");
    distanceRightBack = getSensor("RB");
  }
}


void centerFront() {
  bool isCentered = false;
  int centerthresh = 1; // centerthresh value for alignment accuracy
  int targetCenter = mazeGridSize / 2; // The target center position within a grid

  while (!isCentered) {
    int frontDistance = getSensor("F") + frontSensorOffset; // Get the adjusted front distance
    int positionInGrid = frontDistance % mazeGridSize; // Calculate the position within the grid
    int differenceFromCenter = abs(positionInGrid - targetCenter);

    if (differenceFromCenter <= centerthresh) {
      // If within centerthresh, consider it centered
      isCentered = true;
    } else if (positionInGrid < targetCenter) {
      // If the robot is closer to the front of the grid, move forward
      moveForward(100); // Move a quarter grid size as an example
    } else {
      // If the robot is closer to the back of the grid, move backward
      moveBackward(100); // Move a quarter grid size as an example
    }
    
    // Small delay to allow sensor readings to stabilize
    delay(100);
  }
}

void CEN(){
  center();
  centerFront();
}


void loop() {
  setMotorSpeed(globalSpeed); // Set global speed
  int frontDistance = getSensorDistance(trigPinFront, echoPinFront);
  Serial.println(frontDistance);
  String color = detectColor();
  Serial.println(color);
}
