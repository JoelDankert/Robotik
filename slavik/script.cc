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
#define pinLED 40

#define RED_PIN 51
#define GREEN_PIN 52
#define BLUE_PIN 53


// Global variables for speed control and sensor centerthresh
float globalSpeed = 1; // Speed scale from 0 to 1
#define frontSensorOffset 10 // Example value in centimeters
#define mazeGridSize 40 // Example value in centimeters, adjust as per your maze
//#define timeForward 1000 // Example value in milliseconds, adjust based on your robot's speed and grid size

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

  
  pinMode(pinLED, OUTPUT);

  setColor('X');

  // Initialize color sensor
  if (!tcs.begin()) {
    Serial.println("Couldn't find color sensor");
  }

  dropoff.attach(servopin); // Attaches the servo on pin 9 to the servo object
  dropoff.write(90); // Make sure the servo is at position 0 degrees

  digitalWrite(pinLED, LOW);

  setColor('G');
  delay(100);
  setColor('B');
  delay(100);
  setColor('G');
  delay(100);
  setColor('B');
  delay(100);
  setColor('G');
  delay(100);
  setColor('B');
  delay(100);
  setColor('G');
  delay(100);
  setColor('B');
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
    case 'C':
      digitalWrite(BLUE_PIN, HIGH);
      digitalWrite(GREEN_PIN, HIGH);
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

  if (green*2 < red && blue*2 < red) {
    return "red";
  } else if (red < 10 && blue < 10 && green < 10) {
    return "black";
  } else {
    return "none";
  }
}

void printcolors() {
  uint16_t clear, red, green, blue;
  tcs.getRawData(&red, &green, &blue, &clear);
  Serial.print("Red: ");
  Serial.print(red);
  Serial.print(", Green: ");
  Serial.print(green);
  Serial.print(", Blue: ");
  Serial.print(blue);
  Serial.print(", Clear: ");
  Serial.println(clear);

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
  dropoff.write(0);
  delay(500);
  dropoff.write(90);
  delay(50);
  dropoff.write(180);
  delay(500);
  dropoff.write(90);
}

void tryrightturn(){
  while(getSensor("RF") > 15 && getSensor("F") > 5){
    delay(50);
    if (getSensor("RB") < 15){
      moveForward(50);
    }
    else{
      turnRight(300);
      moveForward(300);
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
  long lastmil2 = 0;

  while (true) { // Infinite loop to keep moving forward
    delay(10);
    setColor('X');
    int distanceRightFront = getSensor("RF");
    int distanceRightBack = getSensor("RB");
    int distanceFront = getSensor("F");
    int averageDistance = (distanceRightFront + distanceRightBack) / 2; // Calculate the average distance to the wall

    long currentmil = millis();
    if (currentmil-lastmil2 >= 2000){
      lastmil2 = currentmil;
      String col = detectColor();
      if(col =="red"){
        if (currentmil-lastmil >= 5000){
          for (int i = 0; i < 25; i++) {
            setColor('G');
            delay(100);
            setColor('X');
            delay(100);
          }
          servodrop();
          lastmil = currentmil;
        }
      }

      if(col =="black"){
        setColor('C');
        turnLeft(300);
      }


    }
    

    if (distanceFront < wallDistance){
      moveBackward(100);
      turnLeft(400);
      continue;
    }
    //if(getSensor("RF") > 15 && getSensor("F") > 5){
    //  tryrightturn();
    //}

    //too close
    if (averageDistance < targetDistance-errorMarginCorrect){
      setColor('B');
      turnLeft(100);
      moveForward(50);
        continue;
    }
    //too far
    if (averageDistance > targetDistance+errorMarginCorrect){
      setColor('B');
      moveForward(100);
      turnRight(50);
        continue; 
    }
    
      setColor('X');
    //straight
    if (abs(distanceRightFront - distanceRightBack) <= errorMargin){
      moveForward(100);
    }
    else if (distanceRightFront > distanceRightBack){
      turnRight(50);
    }
    else{
      turnLeft(50);
    }

         
  }
}



void loop() {
  //TESTSENSORS();
  //printcolors();
  //Serial.print(detectColor());
  FWD();
  //servodrop();
  
  delay(5000);
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

  printcolors();

}
