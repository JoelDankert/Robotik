#include <Adafruit_TCS34725.h>
#include <Servo.h>

// Motor control pins
#define motorPinR1 3
#define motorPinR2 2
#define motorPinL1 4
#define motorPinL2 5
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

#define servopin 9 

#define RED_PIN 51
#define GREEN_PIN 52
#define BLUE_PIN 53

// Global variables for speed control and sensor centerthresh
float globalSpeed = 1; // Speed scale from 0 to 1
#define frontSensorOffset 10 // Example value in centimeters
#define mazeGridSize 40 // Example value in centimeters, adjust as per your maze
//#define timeForward 1000 // Example value in milliseconds, adjust based on your robot's speed and grid size
#define timeTurn 1000 // Example value in milliseconds, adjust based on your robot's speed and grid size
#define timeFwd 1000 // Example value in milliseconds, adjust based on your robot's speed and grid size

Servo dropoff;

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

  dropoff.attach(servopin); // Attaches the servo on pin 9 to the servo object
  dropoff.write(0); // Make sure the servo is at position 0 degrees

  setColor('R');  
  delay(100);
  setColor('G');
  delay(100);
  setColor('B');
  delay(100);
  setColor('R');
  delay(100);
  setColor('G');
  delay(100);
  setColor('B');
  delay(100);
  setColor('R');
  delay(100);
  setColor('G');
  delay(100);
  setColor('B');
  delay(100);
  setColor('R');
  delay(100);
  setColor('G');
  delay(100);
  setColor('B');
  delay(100);
  setColor('R');
  delay(100);
  setColor('G');
  delay(100);
  setColor('B');
  delay(100);
  setColor('X');


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

void servodrop(){
  dropoff.write(90); // Turn servo to 90 degrees
  delay(1000); // Wait for 1 second
  dropoff.write(0); // Return servo to 0 degrees
  delay(1000); // Wait for 1 second
}

void tryrightturn(){
  while(getSensor("RF") > 15 && getSensor("F") > 5){
    delay(50);
    if (getSensor("RB") < 15){
      moveForward(50);
    }
    else{
      turnRight(timeTurn);
      moveForward(timeFwd);
      break;
    }
  }
}

void FWD() {
  int targetDistance = 7; // Target distance from the wall in centimeters
  int wallDistance = 7;
  int errorMargin = 1; // Allowable error margin in centimeters
  int errorMarginCorrect = 2; // Allowable error margin in centimeters
  long lastmil = 0;

  while (true) { // Infinite loop to keep moving forward
    delay(50);
    setColor('X');
    int distanceRightFront = getSensor("RF");
    int distanceRightBack = getSensor("RB");
    int distanceFront = getSensor("F");
    int averageDistance = (distanceRightFront + distanceRightBack) / 2; // Calculate the average distance to the wall

    if(detectColor()=="red"){
      long currentmil = millis();
      if (currentmil-lastmil >= 2500){
        lastmil = currentmil;
        setColor('R');
        delay(100);
        setColor('X');
        delay(100);
        setColor('R');
        delay(100);
        setColor('X');
        delay(100);
        setColor('R');
        delay(100);
        setColor('X');
        delay(100);
        setColor('R');
        delay(100);
        setColor('X');
        delay(100);
        setColor('R');
        delay(100);
        setColor('X');
        servodrop();
      }
    }

    if(detectColor()=="black"){
      turnLeft(2000);
    }

    if (distanceFront < wallDistance){
        turnLeft(timeTurn);
        continue;
    }

    //too close
    if (averageDistance < targetDistance-errorMarginCorrect){
      setColor('B');
      turnLeft(100);
      moveForward(50);
        continue;
    }
    //too far
    if (averageDistance > targetDistance+errorMarginCorrect){
      setColor('G');
      turnRight(100);
      moveForward(100);
        continue; 
    }
    
      setColor('X');
    //straight
    if (abs(distanceRightFront - distanceRightBack) <= errorMargin){
      moveForward(50);
    }
    else if (distanceRightFront > distanceRightBack){
      turnRight(20);
      moveForward(50);
    }
    else{
      turnLeft(20);
      moveForward(50);
    }


  }
}



void loop() {
  // Read distances from each sensor
  FWD();
  //turnRight(500);
  //turnLeft(500);
  delay(1000);
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
