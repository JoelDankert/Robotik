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

#define liposag 1


// Global variables for speed control and sensor centerthresh
float globalSpeed = 1; // Speed scale from 0 to 1
#define frontSensorOffset 10 // Example value in centimeters
#define mazeGridSize 40 // Example value in centimeters, adjust as per your maze

#define timeTurn 500 // Example value in milliseconds, adjust based on your robot's speed and grid size

Servo dropoff;

// Color sensor setup
//Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);

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
  delay(duration*liposag);
  motorsOff();
}

void constForward(){
  digitalWrite(motorPinL1, LOW);
  digitalWrite(motorPinL2, HIGH);
  digitalWrite(motorPinR1, LOW);
  digitalWrite(motorPinR2, HIGH);
}

void moveBackward(int duration) {
  digitalWrite(motorPinL1, HIGH);
  digitalWrite(motorPinL2, LOW);
  digitalWrite(motorPinR1, HIGH);
  digitalWrite(motorPinR2, LOW);
  delay(duration*liposag);
  motorsOff();
}

void turnLeft(int duration) {
  digitalWrite(motorPinL1, HIGH);
  digitalWrite(motorPinL2, LOW);
  digitalWrite(motorPinR1, LOW);
  digitalWrite(motorPinR2, HIGH);
  delay(duration*liposag);
  motorsOff();
}

void turnRight(int duration) {
  digitalWrite(motorPinL1, LOW);
  digitalWrite(motorPinL2, HIGH);
  digitalWrite(motorPinR1, HIGH);
  digitalWrite(motorPinR2, LOW);
  delay(duration*liposag);
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

  if (red < 20 && blue < 20 && green < 20) {
    return "black";
  }
  else if (green*1.4 < red && blue*1.4 < red) {
    return "red";
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

void leftturn(){
  moveForward(200);
  int fronttarget = 4;
  int distanceFront = getSensor("F");
  int f = 0;
  while (abs(distanceFront-fronttarget)>0 && f<15){
    f++;
    delay(100);
     distanceFront = getSensor("F");

    if(abs(distanceFront-fronttarget)==0){
      break;
    }
     
    if (distanceFront < fronttarget || distanceFront >= 100){
     moveBackward(50);
      
    }
    else if (distanceFront > fronttarget){
     moveForward(50);
      
    }

    
  }
  
     
     turnLeft(timeTurn);
     
    
    
}

void FWD() {
  int targetDistance = 6; // Target distance from the wall in centimeters
  int wallDistance = 5;
  int errM = 1;
  int errorMarginCorrect = 1; // Allowable error margin in centimeters
  long lastmil = 0;
  long lastmil2 = 0;
  int distmindiff = 5;
  int currdistdiff = 0;
  int distdiffi = 0;
  int lastdistfront = 0;
  bool constant = false;

  while (true) { // Infinite loop to keep moving forward
    
    if(constant)
    {
      delay(50);
    }
    else
    {
      delay(500);
    }
    
    setColor('X');
    int distanceRightFront = getSensor("RF");
    int distanceRightBack = getSensor("RB");
    int distanceFront = getSensor("F");
    int averageDistance = (distanceRightFront + distanceRightBack) / 2; // Calculate the average distance to the wall


    distdiffi ++;
    currdistdiff += abs(distanceFront - lastdistfront);
    if (distanceFront > 100){
      currdistdiff = 100;
    }
    lastdistfront = distanceFront;
    if(distdiffi>=5){
      distdiffi=0;
      if(currdistdiff < distmindiff){
        setColor('G');
        moveBackward(500);
      }
      
      currdistdiff = 0;
    }

    long currentmil = millis();
    
    if (currentmil-lastmil2 >= 200 || constant){
      String col = detectColor();
      if(col =="red"){
        constant = false;
        moveForward(500);
        if (currentmil-lastmil >= 7000){
          for (int i = 0; i < 25; i++) {
            setColor('G');
            delay(100);
            setColor('X');
            delay(100);
          }
          servodrop();
          lastmil = millis();
        }
      }

      if(col =="black"){
        setColor('B');
        moveBackward(300);
        turnLeft(timeTurn);
        constant = false;
      }
      
      lastmil2 = millis();

    }

    //turn
    //if(getSensor("RF") > 15 && getSensor("F") > 5){
    //  tryrightturn();
    //}

    //walltouch
    if (distanceFront < wallDistance){
      leftturn();
        constant = false;
      
      continue;
    }

    //too far
    if (distanceRightFront > targetDistance+errorMarginCorrect){// || averageDistance > targetDistance+errorMarginCorrect){
        constant = false;
        
      setColor('G');
      moveForward(150);
      delay(50);
      turnRight(150);
        continue; 
    }
    //too close
    if (distanceRightFront < targetDistance-errorMarginCorrect){
        constant = false;
      setColor('B');
      turnLeft(100);
      delay(50);
      moveForward(100);
        continue;
    }
    
    
    
    
    //straight
    if (abs(distanceRightFront - distanceRightBack) <= errM || averageDistance > 15){
      moveForward(200);
      constant=false;
    }
    else 
    {
      int t = 0;
        constant = false;
      while(abs(distanceRightFront - distanceRightBack) > 0){
        t++;
        
        distanceRightFront = getSensor("RF");
        distanceRightBack = getSensor("RB");


        if(abs(distanceRightFront - distanceRightBack) <= 0)
        {
          break;
        }
        
        if (distanceRightFront > distanceRightBack){
          turnRight(50);
        }
        else if (distanceRightFront < distanceRightBack){
          turnLeft(50);
        }
        
        

        
        delay(100);

        if(t >= 15){
        moveForward(100);
        break;
        }
      }
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
