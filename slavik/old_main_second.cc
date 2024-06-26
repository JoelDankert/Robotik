#include <Adafruit_TCS34725.h>
#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>


VL53L0X FF;
VL53L0X RF;
VL53L0X RB;
VL53L0X BB;

// Define the pins connected to the XSHUT (shutdown) pin of each sensor
const int XSHUT_pin_RF = 25; // Example pin for Right Front sensor
const int XSHUT_pin_RB = 26; // Example pin for Right Back sensor
const int XSHUT_pin_F = 24;  // Example pin for Front sensor
const int XSHUT_pin_B = 27;  // Example pin for Back sensor

// Define unique I2C addresses for the sensors
const uint8_t addressRF = 0x30; // Right Front
const uint8_t addressRB = 0x31; // Right Back
const uint8_t addressF = 0x32;  // Front
const uint8_t addressB = 0x33;  // Back

const int Foffset = 0;
const int RBoffset = 0;
const int RFoffset = 0;
const int Boffset = 0;



#define MOTOR1_DIR 4
#define MOTOR1_SPEED 3
#define MOTOR2_DIR 12
#define MOTOR2_SPEED 11
#define MOTOR3_DIR 8
#define MOTOR3_SPEED 5
#define MOTOR4_DIR 7
#define MOTOR4_SPEED 6



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
int timeBetweenScan = 1000;
float timeperangle = 6;

Servo dropoff;

// Color sensor setup
//Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);

void setup() {

  Serial.begin(9600);
  Wire.begin();

  pinMode(XSHUT_pin_F, OUTPUT);
  pinMode(XSHUT_pin_RF, OUTPUT);
  pinMode(XSHUT_pin_RB, OUTPUT);
  pinMode(XSHUT_pin_B, OUTPUT);
  digitalWrite(XSHUT_pin_F, LOW);
  digitalWrite(XSHUT_pin_RF, LOW);
  digitalWrite(XSHUT_pin_RB, LOW);
  digitalWrite(XSHUT_pin_B, LOW);
  delay(50);
  

  delay(50);
  digitalWrite(XSHUT_pin_F, HIGH);
  delay(50);
  FF.setTimeout(500);
  if (!FF.init(true)) {
    Serial.println("Sensor init failed F");
    
  }
  FF.setAddress(addressF);
  
  
  delay(50);
  digitalWrite(XSHUT_pin_RF, HIGH);
  delay(50);
  RF.setTimeout(500);
  if (!RF.init()) {
    Serial.println("Sensor init failed RF");
  }
  RF.setAddress(addressRF);


  delay(50);
  digitalWrite(XSHUT_pin_RB, HIGH);
  delay(50);
  RB.setTimeout(500);
  if (!RB.init()) {
    Serial.println("Sensor init failed RB");
  }
  RB.setAddress(addressRB);


  delay(50);
  digitalWrite(XSHUT_pin_B, HIGH);
  delay(50);
  BB.setTimeout(500);
  if (!BB.init()) {
    Serial.println("Sensor init failed B");
  }
  BB.setAddress(addressB);











  
  

  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR1_SPEED, OUTPUT);
  pinMode(MOTOR2_DIR, OUTPUT);
  pinMode(MOTOR2_SPEED, OUTPUT);
  pinMode(MOTOR3_DIR, OUTPUT);
  pinMode(MOTOR3_SPEED, OUTPUT);
  pinMode(MOTOR4_DIR, OUTPUT);
  pinMode(MOTOR4_SPEED, OUTPUT);


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

  //spin(90);

  motorsOff();  
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
    int Front = getSensor("FF");


    if(detectColor()=="red" && millis()-lastScanMillis > timeBetweenScan){
      lastScanMillis = millis();
      fieldDetect();
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
      moveForward(globalSpeed);
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
      int RB = getSensor("RB");
      if (RB>wallDistanceForRightTurnCheck){
        moveForward(globalSpeed);
        delay(backToFullBackDelay);
        motorsOff();
        spin(90);
        FieldForward();
          
        break;
      }

      moveForward(globalSpeed);
    }
    motorsOff();

}
void LeftTurnSequence(){
  AdjustForward(leftTurnFrontAdjust);
  spin(-90);
}

void blackDetectSequence(){
  moveBackward(globalSpeed);
  delay(blackbackdelay);
  motorsOff();
  spin(-90);
  FieldForward();
}

void FieldForward(){
  int i = 0;
  int startFront = getSensor("FF");
  
  while(i<100){
    i++;
    int currentFront = getSensor("FF");
    if (startFront-fieldSize>currentFront){
      moveForward(globalSpeed);
    }
    else{
      break;
    }
    delay(20);
  }

  motorsOff();
}

void AdjustForward(int goaldist){
  int i = 0;
  while(i<100){
    i++;
    int currentFront = getSensor("FF");

    if (abs(goaldist-currentFront)<errM){
      break;
    }
    
    if (goaldist>currentFront){
      moveForward(globalSpeed);
    }
    else{
      moveBackward(globalSpeed);
    }
    delay(20);
  }
}

bool doRightTurn(int RF, int RB, int FF){
  if (RF > wallDistanceForRightTurnCheck){
    return true;
  }
  return false;
}

bool doLeftTurn(int FF){
  if (FF < wallDistanceForLeftTurnCheck){
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
      turnLeft(globalSpeed);
    } else {
      turnRight(globalSpeed);
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
  int Front = getSensor("FF");
  while (i < 30) {
    i++;

    if (abs(Front - goaldist) <= 1) {
      return;
    }


    if (Front > goaldist) {
      moveForward(globalSpeed);
    } else {
      moveBackward(globalSpeed);
    }
    delay(30);
    motorsOff();

    delay(100);
    Front = getSensor("FF");
  }
}

void spinNOGYRO(int angle){
  if (angle > 0){
    turnRight(globalSpeed);
    delay(angle*timeperangle);
  }
  else{
    turnLeft(globalSpeed);
    delay(angle*timeperangle);
  }

  motorsOff();
}

void spin(int angle) {

  spinNOGYRO(angle); //REMOVE WHEN GYRO
  return;

  int currentAngle = getGyroAngle();
  
  if (currentAngle < -180)
  {
    currentAngle+=360;
  }
  if (currentAngle > 180)
  {
    currentAngle-=360;
  }
  
  int targetAngle = currentAngle + angle;

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
  
    
    int tospin = targetAngle - currentAngle;

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
      turnLeft(globalSpeed);
    }
    if (tospin > 0){
      turnRight(globalSpeed);
    }
    
  }
  motorsOff();

  
}


// MOTORFUNCS
void setMotorSpeed(float speed) {
  int pwmValue = speed * 255;
  analogWrite(MOTOR1_SPEED, pwmValue);
  analogWrite(MOTOR2_SPEED, pwmValue);
  analogWrite(MOTOR3_SPEED, pwmValue);
  analogWrite(MOTOR4_SPEED, pwmValue);
}

void setMotorSpeed1(float speed) { analogWrite(MOTOR1_SPEED, speed * 255); }

void setMotorSpeed2(float speed) { analogWrite(MOTOR2_SPEED, speed * 255); }

void setMotorSpeed3(float speed) { analogWrite(MOTOR3_SPEED, speed * 255); }

void setMotorSpeed4(float speed) { analogWrite(MOTOR4_SPEED, speed * 255); }

void moveForward(float spd) {
  setMotorSpeed(spd);
  digitalWrite(MOTOR1_DIR, LOW);
  digitalWrite(MOTOR2_DIR, LOW);
  digitalWrite(MOTOR3_DIR, LOW);
  digitalWrite(MOTOR4_DIR, HIGH);
}

void moveBackward(float spd) {
  setMotorSpeed(spd);
  digitalWrite(MOTOR1_DIR, HIGH);
  digitalWrite(MOTOR2_DIR, HIGH);
  digitalWrite(MOTOR3_DIR, HIGH);
  digitalWrite(MOTOR4_DIR, LOW);
}

void turnLeft(float spd) {
  setMotorSpeed(spd);
  digitalWrite(MOTOR1_DIR, HIGH);
  digitalWrite(MOTOR2_DIR, LOW);
  digitalWrite(MOTOR3_DIR, HIGH);
  digitalWrite(MOTOR4_DIR, HIGH);
}

void turnRight(float spd) {
  setMotorSpeed(spd);
  digitalWrite(MOTOR1_DIR, LOW);
  digitalWrite(MOTOR2_DIR, HIGH);
  digitalWrite(MOTOR3_DIR, LOW);
  digitalWrite(MOTOR4_DIR, LOW);
}

void motorsOff() {
  analogWrite(MOTOR1_SPEED, 0);
  analogWrite(MOTOR2_SPEED, 0);
  analogWrite(MOTOR3_SPEED, 0);
  analogWrite(MOTOR4_SPEED, 0);
}

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


int getGyroAngle() {
  return 0;
}


//OTHER
void fieldDetect() {

  setColor('R');
  delay(5000);
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
  int distanceLeft = getSensor("FF");
  int distanceRightFront = getSensor("RF");
  int distanceRightBack = getSensor("RB");
  int distanceFront = getSensor("BB");

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
      // If the input is not R, G, or BB, do nothing (all LEDs remain off)
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





int getSensor(String sensorID) {
  if (sensorID == "FF") {
    return FF.readRangeSingleMillimeters()+Foffset;
  } else if (sensorID == "RF") {
    return RF.readRangeSingleMillimeters()+RFoffset;
  } else if (sensorID == "RB") {
    return RB.readRangeSingleMillimeters()+RBoffset;
  } else if (sensorID == "BB") {
    return BB.readRangeSingleMillimeters()+Boffset;
  } else {
    Serial.println("Invalid sensor ID");
    return -1; // Indicate an error
  }
}
