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
const int Boffset = -4;


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

int rightWallDistanceMax = 10;
int rightWallDistanceAdj = 8;
int frontWallDistanceMin = 10;
int frontWallDistanceGoal = 15;
int tryWallDistanceGoal = 10;
float globalSpeed = 1;
int fieldSize = 30;
float timeperangle = 6;
float speedAdj = 0.2;


const float Frightturn = 0.1;
const float Fleftturnspeed = 0.5;


int state = 0;
float EDcurrentchange = 10;
float EDresetvalue = 10;
int EDlast = 0;
float EDcap = 3;
float EDchangeslow = 5;


Servo dropoff;

// Color sensor setup
//Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);




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
  

  delay(50);
  digitalWrite(XSHUT_pin_F, HIGH);
  delay(50);
  FF.setTimeout(500);
  if (!FF.init(true)) {
    Serial.println("F Sensor init failed");
    
  }
  FF.setAddress(addressF);
  
  
  delay(50);
  digitalWrite(XSHUT_pin_RF, HIGH);
  delay(50);
  RF.setTimeout(500);
  if (!RF.init()) {
    Serial.println("RF Sensor init failed");
  }
  RF.setAddress(addressRF);


  delay(50);
  digitalWrite(XSHUT_pin_RB, HIGH);
  delay(50);
  RB.setTimeout(500);
  if (!RB.init()) {
    Serial.println("RB Sensor init failed");
  }
  RB.setAddress(addressRB);


  delay(50);
  digitalWrite(XSHUT_pin_B, HIGH);
  delay(50);
  BB.setTimeout(500);
  if (!BB.init()) {
    Serial.println("B Sensor init failed");
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

  //motorsOff();  
  //TESTSENSORS();
  //printcolors();
  //Serial.print(detectColor());
  MAIN();
  
  
  //servodrop();

  delay(1000);
}

void MAIN() {
  while(true){
    delay(50);


    Serial.println(state);                                                                                              //COLOR LED
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

                                                                                                            //LEFT TURNS (#LT)
    int spd = 1;
    int front = getSensor("FF");
    int i = 0;
    while(front < frontWallDistanceGoal && i < 30){
      i++;
      turnLeft(Fleftturnspeed);
      if (state == -1){ state = 3; }
      
      delay(10);
      front = getSensor("FF");
    }




                                                                                                            //RIGHT WALL COMPENSATION (#WC)
    float rightF = getSensor("RF");
    float rightB = getSensor("RB");

    float diff = rightF-rightB;
    if (rightF <= tryWallDistanceGoal && rightB <= tryWallDistanceGoal){
      moveForward(1);
      if (state == -1){ state = 1; }
      float compamount = max(min(   (abs(diff)/3   )   ,1),0.2);
      float spdC = -0.8*compamount +1.2;
      if (diff > 0){
        setMotorSpeedR(spdC);
      }
      else{
      
        setMotorSpeedL(spdC);

        if (rightF > rightWallDistanceAdj){
          setMotorSpeedL(speedAdj);
        }
        
      }

      

      
    }
    

                                                                                                            //RIGHT TURNS (#RT)
    rightF = getSensor("RF");
    //rightB = getSensor("RB");
     
    if (rightF > rightWallDistanceMax){
      if (state == -1){ state = 2; }
      moveForward(1);
      setMotorSpeedR(Frightturn);
    }

    
    if (errordetecttick(front)){
      
    }



    
  }
}




bool errordetecttick(int front){                                                                            //ERROR DETECT (#ED)

  float absolutechange = abs(front-EDlast);
  EDcurrentchange = (EDcurrentchange*EDchangeslow + absolutechange)/(EDchangeslow+1);


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




                                                                                                            //TOFS

float getSensor(String sensorID) {
  if (sensorID == "FF") {
    int v = FF.readRangeSingleMillimeters();
    if(v > 1000){
      return 0;
    }
    return v/10+Foffset;
  } else if (sensorID == "RF") {
    int v = RF.readRangeSingleMillimeters();
    if(v > 1000){
      return 0;
    }
    return v/10+RFoffset;
  } else if (sensorID == "RB") {
    int v = RB.readRangeSingleMillimeters();
    if(v > 1000){
      return 0;
    }
    return v/10+RBoffset;
  } else if (sensorID == "BB") {
    int v = BB.readRangeSingleMillimeters();
    if(v > 1000){
      return 0;
    }
    return v/10+Boffset;
    return BB.readRangeSingleMillimeters()/10+Boffset;
  } else {
    Serial.println("Invalid sensor ID");
    return -1; // Indicate an error
  }
}


