#include <Adafruit_TCS34725.h>
#include <Servo.h>

// Motor control pins
#define motorPinL1 3
#define motorPinL2 2
#define motorPinR1 4
#define motorPinR2 5
#define motorSpeedL 6
#define motorSpeedR 7

// Ultrasonic sensor pins
#define echoPinFront 23
#define trigPinFront 22
#define echoPinLeft 29
#define trigPinLeft 28
#define echoPinRightFront 27
#define trigPinRightFront 26
#define echoPinRightBack 25
#define trigPinRightBack 24

#define RED_PIN 51
#define GREEN_PIN 52
#define BLUE_PIN 53

// Global variables for speed control and sensor centerthresh
float globalSpeed = 1; // Speed scale from 0 to 1
#define frontSensorOffset 10 // Example value in centimeters
#define mazeGridSize 40 // Example value in centimeters, adjust as per your maze
//#define timeForward 1000 // Example value in milliseconds, adjust based on your robot's speed and grid size
#define timeTurn 1000 // Example value in milliseconds, adjust based on your robot's speed and grid size


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
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  setColor('X');

  // Initialize color sensor
  if (!tcs.begin()) {
    Serial.println("Couldn't find color sensor");
  }
}



void setColor(char color) {
  // Turn all LEDs off initially
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, LOW);
  
  // Turn on the selected LED
  switch(color) {
    case 'R':
      digitalWrite(RED_PIN, HIGH);
      break;
    case 'G':
      digitalWrite(GREEN_PIN, HIGH);
      break;
    case 'B':
      digitalWrite(BLUE_PIN, HIGH);
      break;
    default:
      // If the input is not R, G, or B, do nothing (all LEDs remain off)
      break;
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
  motorsOff();
}

void moveBackward(int duration) {
  digitalWrite(motorPinL1, HIGH);
  digitalWrite(motorPinL2, LOW);
  digitalWrite(motorPinR1, HIGH);
  digitalWrite(motorPinR2, LOW);
  delay(duration);
  motorsOff();
}

void turnLeft(int duration) {
  digitalWrite(motorPinL1, HIGH);
  digitalWrite(motorPinL2, LOW);
  digitalWrite(motorPinR1, LOW);
  digitalWrite(motorPinR2, HIGH);
  delay(duration);
  motorsOff();
}

void turnRight(int duration) {
  digitalWrite(motorPinL1, LOW);
  digitalWrite(motorPinL2, HIGH);
  digitalWrite(motorPinR1, HIGH);
  digitalWrite(motorPinR2, LOW);
  delay(duration);
  motorsOff();
}

void motorsOff(){
  digitalWrite(motorPinL1, LOW);
  digitalWrite(motorPinL2, LOW);
  digitalWrite(motorPinR1, LOW);
  digitalWrite(motorPinR2, LOW);
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


void RightTurnSequence(){
  setColor('G');
  delay(5000);
  //moveForward(100);
  //turnright(100);
  //moveForeward(100);
}

void FWD() {
  int targetDistance = 5; // Target distance from the wall in centimeters
  int wallDistance = 3;
  int errorMargin = 1; // Allowable error margin in centimeters
  int errorMarginCorrect = 1; // Allowable error margin in centimeters
  int turnDistance = 20; // Allowable error margin in centimeters

  while (true) { // Infinite loop to keep moving forward
    delay(1000);
    int distanceRightFront = getSensor("RF");
    int distanceRightBack = getSensor("RB");
    int distanceFront = getSensor("F");
    int averageDistance = (distanceRightFront + distanceRightBack) / 2; // Calculate the average distance to the wall


    //if (distanceRightFront > turnDistance){
    //    RightTurnSequence();
    //    continue;
    //}
    
    if (distanceFront < wallDistance){
        turnLeft(100);
        continue;
    }

    //too close
    if (distanceRightFront < targetDistance-errorMarginCorrect){
      turnLeft(100);
      setColor('B');
    }
    if (distanceRightFront > targetDistance+errorMarginCorrect){
      turnRight(100);
      setColor('B');
    }
    
    //straight
    if (abs(distanceRightFront - distanceRightBack) <= errorMargin){
      moveForward(100);
    }
    else if (distanceRightFront > distanceRightBack){
      turnRight(100);
      moveForward(100);
    }
    else{
      turnLeft(100);
      moveForward(100);
    }


  }
}



void loop() {
  // Read distances from each sensor
  FWD();
  //TESTSENSORS();
  // Delay for a bit before reading again to make the output readable
  //delay(1000); // 1-second delay
}


void TESTSENSORS(){
  int distanceLeft = getSensor("L");
  int distanceRightFront = getSensor("RF");
  int distanceRightBack = getSensor("RB");
  int distanceFront = getSensor("F");

  // Write the distances to the serial port
  Serial.print("Left: ");
  Serial.print(distanceLeft);
  Serial.print(" cm, Right Front: ");
  Serial.print(distanceRightFront);
  Serial.print(" cm, Right Back: ");
  Serial.print(distanceRightBack);
  Serial.print(" cm, Front: ");
  Serial.print(distanceFront);
  Serial.println(" cm");

}
