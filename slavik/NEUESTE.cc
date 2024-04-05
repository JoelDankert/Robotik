#include <Adafruit_TCS34725.h>
#include <Servo.h>

// Motor control pins
#define motorPinR1 3
#define motorPinR2 2
#define motorPinL1 5
#define motorPinL2 4
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


int wallDistanceForRightTurnCheck = 15;
int wallDistanceForLeftTurnCheck = 5;
int leftTurnFrontAdjust = 10;
int backToFullBackDelay = 100;
int goalWallDistance = 5;
int maxWallDistance = 8;
int minWallDistance = 3;
int adjustSpinAngle = 20;
int errM = 0;
float globalSpeed = 1;
int fieldSize = 30;
int slowSpinRange = 5;
int slowSpinSpeed = 0.5;
int lastScanMillis = 0;
int blackbackdelay = 100;

Servo dropoff;

// Color sensor setup
//Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);

void setup() {

  //TODO SETUP FIELDSIZE
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

  dropoff.attach(servopin);  // Attaches the servo on pin 9 to the servo object
  dropoff.write(90);         // Make sure the servo is at position 0 degrees

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

void loop() {
  //TESTSENSORS();
  //printcolors();
  //Serial.print(detectColor());
  MAIN();
  //servodrop();

  delay(5000);
}

void MAIN() {
  
  while (true){
    delay(100);
    int RightFront = getSensor("RF");
    int RightBack = getSensor("RF");
    int Front = getSensor("F");


    if(detectColor()=="red" && millis()-lastScanMillis > timeBetweenScan){
      lastScanMillis = millis();
      dropoff();
    }
    if(detectColor()=="black"){
      setColor('C');
      blackDetectSequence();
    }


    //activeTurns
    if(doRightTurn(RightFront,RightBack,Front)){
      RightTurnSequence();
      continue;
    }
    if(doLeftTurn(Front)){
      LeftTurnSequence();
      continue;
    }

    if (tooCloseRight(RightFront)){
      adjustDistanceRight();
      continue;
    }
    else if (angleAdjustmentNeeded(RightFront,RightBack)){
      adjustAngleRight();
      continue;
    }
    else{
      moveForward();
      continue;
    }

    


    //TODO









  }
}


void RightTurnSequence(){
    
    int i = 0;
    while(i<50)
    {
      i++;
      delay(50);
      RB = getSensor("RB");
      if (RB>wallDistanceForRightTurnCheck){
        moveForward();
        delay(backToFullBackDelay);
        stopMotors();
        spin(90);
        FieldForward();
          
        break;
      }

      moveForward();
    }
    stopMotors();

}
void LeftTurnSequence(){
  AdjustForward(leftTurnFrontAdjust);
  spin(-90);
}

void blackDetectSequence(){
  moveBackward();
  delay(blackbackdelay);
  stopMotors();
  spin(-90);
  FieldForward();
}

void FieldForward(){
  int i = 0;
  startFront = getSensor("F");
  
  while(i<100){
    i++;
    currentFront = getSensor("F");
    if (startFront-fieldSize>currentFront){
      moveForward();
    }
    else{
      break;
    }
    delay(20);
  }

  stopMotors();
}

void AdjustForward(goaldist){
  while(i<100){
    i++;
    currentFront = getSensor("F");

    if (abs(goal-currentFront)<errM){
      break
    }
    
    if (goal>currentFront){
      moveForward();
    }
    else{
      moveBackward();
    }
    delay(20);
  }
}

bool doRightTurn(int RF, int RB, int F){
  if (RF > wallDistanceForRightTurnCheck){
    return true;
  }
  return false;
}

bool doLeftTurn(int F){
  if (F < wallDistanceForLeftTurnCheck){
    return true;
  }
  return false;
}

bool tooCloseRight(int RF){
  if (RF < minWallDistance){
    return true;
  }
  return false;
}
bool tooFarRight(int RF){
  if (RF > maxWallDistance){
    return true;
  }
  return false;
}

bool angleAdjustmentNeeded(int RF, int RB){
  if (abs(RF-RB)<errM){
    return false;
  }
  return true;
}

void adjustAngleRight() {
  int i = 0;
  int RBack = getSensor("RB");
  int RFront = getSensor("RF");
  while (i < 30) {
    i++;

    if (abs(RBack - RFront) <= 1) {
      return;
    }


    if (RBack > RFront) {
      turnLeft();
    } else {
      turnRight();
    }
    delay(30);
    motorsOff();

    delay(100);

    RBack = getSensor("RB");
    RFront = getSensor("RF");
  }
}

void adjustDistanceRight() {
    spin(-adjustSpinAngle);
}

void adjustDistanceFront(int goaldist) {
  int i = 0;
  int Front = getSensor("F");
  while (i < 30) {
    i++;

    if (abs(Front - goaldist) <= 1) {
      return;
    }


    if (Front > goaldist) {
      moveForward();
    } else {
      moveBackward();
    }
    delay(30);
    motorsOff();

    delay(100);
    Front = getSensor("F");
  }
}

void spin(int angle) {
  currentAngle = getGyroAngle();
  
  if (currentAngle < -180)
  {
    currentAngle+=360;
  }
  if (currentAngle > 180)
  {
    currentAngle-=360;
  }
  
  targetAngle = currentAngle + angle;

  if (targetAngle < -180)
  {
    targetAngle+=360;
  }
  if (targetAngle > 180)
  {
    targetAngle-=360;
  }

  int i = 0;

  while(i<30)
  {
    i++;
    delay(50);
    currentAngle = getGyroAngle();

    if (currentAngle < -180)
      {
        currentAngle+=360;
      }
      if (currentAngle > 180)
      {
        currentAngle-=360;
      }
  
    
    tospin = targetAngle - currentAngle;

    // Adjust for circular nature of angles
    if (tospin > 180) {
        tospin -= 360;
    }
    else if (tospin < -180) {
        tospin += 360;
    }

    if (abs(tospin) < slowSpinRange){
      setMotorSpeed(slowSpinSpeed);
    }
    else{
      setMotorSpeed(globalSpeed);
    }
    if (abs(tospin) < errM){
      break;
    }
    if (tospin < 0){
      turnLeft();
    }
    if (tospin > 0){
      turnRight();
    }
    
  }
  motorsOff();

  
}


// MOTORFUNCS
void setMotorSpeed(float speed) {
  analogWrite(motorSpeedL, speed * 255);
  analogWrite(motorSpeedR, speed * 255);
}
void setMotorSpeedR(float speed) {
  analogWrite(motorSpeedR, speed * 255);
}
void setMotorSpeedL(float speed) {
  analogWrite(motorSpeedL, speed * 255);
}

void moveForward() {
  digitalWrite(motorPinL1, LOW);
  digitalWrite(motorPinL2, HIGH);
  digitalWrite(motorPinR1, LOW);
  digitalWrite(motorPinR2, HIGH);
}

void moveBackward() {
  digitalWrite(motorPinL1, HIGH);
  digitalWrite(motorPinL2, LOW);
  digitalWrite(motorPinR1, HIGH);
  digitalWrite(motorPinR2, LOW);
}

void turnLeft() {
  digitalWrite(motorPinL1, HIGH);
  digitalWrite(motorPinL2, LOW);
  digitalWrite(motorPinR1, LOW);
  digitalWrite(motorPinR2, HIGH);
}

void turnRight() {
  digitalWrite(motorPinL1, LOW);
  digitalWrite(motorPinL2, HIGH);
  digitalWrite(motorPinR1, HIGH);
  digitalWrite(motorPinR2, LOW);
}

void motorsOff() {
  digitalWrite(motorPinL1, LOW);
  digitalWrite(motorPinL2, LOW);
  digitalWrite(motorPinR1, LOW);
  digitalWrite(motorPinR2, LOW);
}


//SENSORFUNCS
String detectColor() {
  uint16_t clear, red, green, blue;
  tcs.getRawData(&red, &green, &blue, &clear);

  if (red < 20 && blue < 20 && green < 20) {
    return "black";
  } else if (green * 1.4 < red && blue * 1.4 < red) {
    return "red";
  } else {
    return "none";
  }
}

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
  return (duration * 0.034 / 2) * conversionvalue;
}

int getGyroAngle() {
  return 0;
}


//OTHER
void fieldDetect() {

  setColor('R')
  delay(5000)
  setColor('X');
  
  dropoff.write(0);
  delay(500);
  dropoff.write(90);
  delay(50);
  dropoff.write(180);
  delay(500);
  dropoff.write(90);
}



//DEBUG
void TESTSENSORS() {
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

void setColor(char color) {
  // Turn all LEDs off initially
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, LOW);

  // Turn on the selected LED
  switch (color) {
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

void printcolors() {
  uint16_t clear, red, green, blue;
  tcs.getRawData(&red, &green, &blue, &clear);
  Serial.print("Red: ");
  Serial.print(red);
  Serial.print(", Green: ");
  Serial.print(green);
  Serial.print(", Blue: ");
  Serial.print(blue);
}
