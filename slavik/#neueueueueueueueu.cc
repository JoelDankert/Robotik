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

const int Foffset = -1;
const int RBoffset = -1;
const int RFoffset = -1;
const int Boffset = -1;


#define MOTOR1_DIR 4
#define MOTOR1_SPEED 3
#define MOTOR2_DIR 12
#define MOTOR2_SPEED 11
#define MOTOR3_DIR 8
#define MOTOR3_SPEED 5
#define MOTOR4_DIR 7
#define MOTOR4_SPEED 6

#define redPin 29    // Pin connected to the red signal from the Nano
#define blackPin 30  // Pin connected to the black signal from the Nano
#define resetPin 31  // Output pin to send reset signal to the Nano



#define servopin 22
#define pinLED 40

#define RED_PIN 51
#define GREEN_PIN 52
#define BLUE_PIN 53

void(* resetFunc) (void) = 0;


int rightWallDistanceMax = 10;
int frontWallDistanceMin = 10;
int frontWallDistanceGoal = 15;
int tryWallDistanceGoal = 15;
int rightWallDistanceGoal = 6;

float globalSpeed = 1;
int fieldSize = 30;
float timeperangle = 6;
float speedAdj = 0.2;


int rightturncancel = 15;


const float Frightturn = 0.2;
const float Fleftturnspeed = 1;




int state = 0;
float EDcurrentchange = 10;
float EDresetvalue = 30;
int EDlast = 0;
float EDcap = 5;
float EDchangeslow = 10;

int fatalerrorcount = 0;
int fatalerrorreset = 100;



float lastFront = 0;
float frontmax = 3;


Servo dropoff;





                                                                                                            //SETUP
void setup() {


  Serial.begin(9600);
  Wire.begin();



  Serial.println("START");

  
  pinMode(XSHUT_pin_F, OUTPUT);
  pinMode(XSHUT_pin_RF, OUTPUT);
  pinMode(XSHUT_pin_RB, OUTPUT);
  pinMode(XSHUT_pin_B, OUTPUT);
  digitalWrite(XSHUT_pin_F, LOW);
  digitalWrite(XSHUT_pin_RF, LOW);
  digitalWrite(XSHUT_pin_RB, LOW);
  digitalWrite(XSHUT_pin_B, LOW);
  delay(50);
  

  initializeSensors();

  

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

  pinMode(redPin, INPUT);
  pinMode(blackPin, INPUT);
  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, HIGH);



  


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

  //motorsOff();  
  //TESTSENSORS();
  //Serial.print(detectColor());
  //delay(3000);
  //ResetSensors();
  MAIN();
  
  
  //servodrop();

  delay(1000);
}

void MAIN() {
  while(true){
    delay(10);                                                                                            //COLOR LED
    if (state == 0){
      setColor('X');
    }
    else if (state == 1){
      setColor('G');
    }
    else if (state == 2){
      setColor('B');
    }
    else if (state == 3){
      setColor('R');
    }
    state = -1;



    String det = "";
    det = detectColor();                                                                                                              //COLOR DETECTION ACTIONS (#CD)
    if (det == "Red"){
      fieldDetect();
    }
    if (det == "black"){
      turnLeft(1);
      delay(200);
      motorsOff();
    }




                                                                                                            //LEFT TURNS (#LT)
    int spd = 1;
    int front = getSensor("FF");
    int i = 0;
    while(front < frontWallDistanceGoal && i < 30){
      i++;
      turnLeft(Fleftturnspeed);
      if (state == -1){ state = 3; setColor('R');}

      bool skip = false;
      if (front > lastFront+frontmax){
        setColor('G');
        moveBackward(1);
        delay(200);
        turnLeft(Fleftturnspeed);
        delay(300);
        skip = true;
      }
      delay(20);
      lastFront = front;
      
      front = getSensor("FF");
      if (skip){
        lastFront = front;
      }




      if (errordetecttick(front)){
        moveBackward(1);
        delay(500);
        motorsOff();
      }
    }




                                                                                                            //RIGHT WALL COMPENSATION (#WC)
    float rightF = getSensor("RF");
    float rightB = getSensor("RB");

    float distadj = (rightF - rightWallDistanceGoal)*0.7;

    float diff = rightF-rightB+distadj;
    if (rightF <= tryWallDistanceGoal && rightB <= tryWallDistanceGoal){
      moveForward(1);
      if (state == -1){ state = 1; setColor('G');}

      float compamount = max(min(   (pow(abs(diff),2)*0.1)   ,1),0);
      float spdC = -1*compamount +1;

      if (diff > 0){
        setMotorSpeedR(spdC);
      }
      else{
        setMotorSpeedL(spdC);
      }

      

      
    }
    

                                                                                                            //RIGHT TURNS (#RT)
    rightF = getSensor("RF");
    front = getSensor("FF");
     
    if (rightF > rightWallDistanceMax){
      if (state == -1){ state = 2; setColor('B');}
      if(front < rightturncancel){
        setColor('R');
      }
      else{
        moveForward(1);
        setMotorSpeedR(Frightturn);
      }
    }





    
    if (errordetecttick(front)){
      moveBackward(1);
      delay(500);
      motorsOff();
    }



    
  }
}




bool errordetecttick(int front){                                                                            //ERROR DETECT (#ED)



  float absolutechange = abs(front-EDlast);
  EDcurrentchange = (EDcurrentchange*EDchangeslow + absolutechange)/(EDchangeslow+1);


  if (EDcurrentchange > EDresetvalue){
    EDcurrentchange = EDresetvalue;
  }
  if (EDcurrentchange <= EDcap){
    EDcurrentchange = EDresetvalue;
    return true;
  }
 
  EDlast = front;

  
  return false;
}









                                                                                                            //MOTORFUNCS
void setMotorSpeed(float speed) {
  int pwmValue = speed * 255;
  analogWrite(MOTOR1_SPEED, pwmValue);
  analogWrite(MOTOR2_SPEED, pwmValue);
  analogWrite(MOTOR3_SPEED, pwmValue);
  analogWrite(MOTOR4_SPEED, pwmValue);
}

void setMotorSpeedL(float speed) { analogWrite(MOTOR1_SPEED, speed * 255); analogWrite(MOTOR3_SPEED, speed * 255);}

void setMotorSpeedR(float speed) { analogWrite(MOTOR2_SPEED, speed * 255); analogWrite(MOTOR4_SPEED, speed * 255);}

void moveForward(float spd) {
  if (spd != -1){
  setMotorSpeed(spd);
    
  }
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

                                                                                                            //COLOR DETECTION (#CD)
String detectColor() {
  return "none";

  bool redSignal = digitalRead(redPin) == HIGH;
  bool blackSignal = digitalRead(blackPin) == HIGH;

  if (redSignal && !blackSignal) {
    resetSignal();
    return "Red";
  } else if (blackSignal && !redSignal) {
    resetSignal();
    return "Black";
  } else if (blackSignal && redSignal){
    resetSignal();
    return "Both";
  }
  return "none";  // No color detected
}

void resetSignal() {
  motorsOff();
  digitalWrite(resetPin, LOW);  // Send a low signal to reset
  delay(100);                    // Hold the signal for 100 milliseconds
  digitalWrite(resetPin, HIGH);  // Return to high
}

                                                                                                            //DROPOFF SYSTEM (#DO)
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
  int distanceFront = getSensor("FF");
  int distanceRightFront = getSensor("RF");
  int distanceRightBack = getSensor("RB");
  int distanceBack = getSensor("BB");

  // Write the distances to the serial port
  Serial.print("Front: ");
  Serial.print(distanceFront);
  Serial.print(" cm, Right Front: ");
  Serial.print(distanceRightFront);
  Serial.print(" cm, Right Back: ");
  Serial.print(distanceRightBack);
  Serial.print(" cm, Back: ");
  Serial.print(distanceBack);
  Serial.println(" cm");


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





                                                                                                            //TOFS

float getSensor(String sensorID) {
  fatalerrorcount-=1;
  if (fatalerrorcount > fatalerrorreset){
    ResetSensors();
  }


  if (sensorID == "FF") {
    int v = FF.readRangeSingleMillimeters();
    if(v <= 1 ){
      fatalerrorcount+=10;
      return 1000;
    }
    

    float returnval = v/10+Boffset;
    return returnval;



  } else if (sensorID == "RF") {
    int v = RF.readRangeSingleMillimeters();
    if(v <= 1 ){
      fatalerrorcount+=10;
      return 1000;
    }

    float returnval = v/10+Boffset;
    return returnval;

  } else if (sensorID == "RB") {
    int v = RB.readRangeSingleMillimeters();
    if(v <= 1 ){
      fatalerrorcount+=10;
      return 1000;
    }
    

    float returnval = v/10+Boffset;
    return returnval;

  } else if (sensorID == "BB") {
    int v = BB.readRangeSingleMillimeters();
    if(v <= 1 ){
      //fatalerrorcount+=10;
      return 1000;
    }


    float returnval = v/10+Boffset;
    return returnval;


  } else {
    Serial.println("Invalid sensor ID");
    return 1000; // Indicate an error
  }
}

void ResetSensors(){
  setColor('X');
  resetFunc();
}



void initializeSensors(){
  delay(50);
  digitalWrite(XSHUT_pin_F, HIGH);
  delay(50);
  if (!FF.init(true)) {
    Serial.println("F Sensor init failed");
    
  }
  FF.setAddress(addressF);
  
  
  delay(50);
  digitalWrite(XSHUT_pin_RF, HIGH);
  delay(50);
  if (!RF.init()) {
    Serial.println("RF Sensor init failed");
  }
  RF.setAddress(addressRF);


  delay(50);
  digitalWrite(XSHUT_pin_RB, HIGH);
  delay(50);
  if (!RB.init()) {
    Serial.println("RB Sensor init failed");
  }
  RB.setAddress(addressRB);


  delay(50);
  digitalWrite(XSHUT_pin_B, HIGH);
  delay(50);
  if (!BB.init()) {
    Serial.println("B Sensor init failed");
  }
  BB.setAddress(addressB);
}

