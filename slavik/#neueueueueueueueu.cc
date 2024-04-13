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
int frontWallDistanceGoal = 15;
int tryWallDistanceGoal = 15;
int rightWallDistanceGoal = 8;

float globalSpeed = 1;
int fieldSize = 30;
float timeperangle = 6;
float speedAdj = 0.2;


int rightturncancel = 15;


const float Frightturn = 0.1;
const float Fleftturnspeed = 1;




int state = 0;
float EDcurrentchange = 30;
float EDresetvalue = 30;
int EDlast = 0;
float EDcap = 3;
float EDchangeslow = 10;

int fatalerrorcount = 0;
int fatalerrorreset = 100;



float lastFront = 0;
float frontmax = 2;


//forblinktick
unsigned long previousMillis = 0;  // will store last time LED was updated
const long onTime = 25;           // milliseconds of on-time
const long offTime = 100;          // milliseconds of off-time
bool ledState = false;


Servo dropoff;





void setup() {                                                                                              //SETUP


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
    delay(10);

    if (state == 0 || state == -1){                                                                                            //RESETLED
      setColor('X');
    }
    state = -1;






    String det = "";
    det = detectColor();                                                                                                           //COLOR DETECTION ACTIONS (#CD)
    if (det == "Red"){
      fieldDetect();
    }
    if (det == "black"){
      turnLeft(1);
      delay(200);
      motorsOff();
    }



    int spd = 1;                                                                                                                    //LEFT TURNS (#LT)
    int front = getSensor("FF");
    float rightF = getSensor("RF");
    float rightB = getSensor("RB");

    if (errordetecttick(front)){                                                                                                    //COLORDETECTTICK (#CDT)
      moveBackward(1);
      setColor('W');
      delay(500);
      motorsOff();
    }

    int i = 0;
    while(front < frontWallDistanceGoal && i < 30){
      i++;
      turnLeft(Fleftturnspeed);
      if (state == -1){ state = 3; setColor('Y');}

      bool skip = false;
      if (front > lastFront+frontmax){                                                                                             //Suboptimal Left Turn Quantification Compensator
        setColor('W');
        moveBackward(1);
        delay(100);
        turnLeft(Fleftturnspeed);
        delay(200);
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
    if (state!=-1){
      continue;
    }



     
                                                                                                                                            //RIGHT TURNS (#RT)

    
     
    if (rightF > rightWallDistanceMax){
      if (state == -1){ state = 2; setColor('Y');}
      if(front < rightturncancel){
        
      }
      else{
        moveForward(1);
        setMotorSpeedR(Frightturn);
      }
    }
    if (state!=-1){
      continue;
    }

                                                                                                      //Exponential Wall-Alignment Righting Mechanism EWARM (#WC)
    

    float distadj = (rightF - rightWallDistanceGoal)*0.7;

    float diff = rightF-rightB+distadj;
    if (rightF <= tryWallDistanceGoal && rightB <= tryWallDistanceGoal){
      moveForward(1);
      if (state == -1){ state = 1; setColor('G');}

      float compamount = max(min(   (pow(abs(diff),2)*0.1)   ,1),0);
      float spdC = -1*compamount +1;
      if (compamount < 0.3){
        BLINKontrack();
      }

      if (diff > 0){
        setMotorSpeedR(spdC);
      }
      else{
        setMotorSpeedL(spdC);
      }
      
    }
    if (state!=-1){
      continue;
    }





    if (state == -1){
      moveForward(1);
    }





    
    



    
  }
}


bool errordetecttick(int front){                                                                            //Progressive Stasis Error Detection PSED (#ED)



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


void setMotorSpeed(float speed) {                                                                                      //MOTORFUNCS
  int pwmValue = speed * 255 * globalSpeed;
  analogWrite(MOTOR1_SPEED, pwmValue);
  analogWrite(MOTOR2_SPEED, pwmValue);
  analogWrite(MOTOR3_SPEED, pwmValue);
  analogWrite(MOTOR4_SPEED, pwmValue);
}

void setMotorSpeedL(float speed) { analogWrite(MOTOR1_SPEED, speed * 255 * globalSpeed); analogWrite(MOTOR3_SPEED, speed * 255 * globalSpeed);}

void setMotorSpeedR(float speed) { analogWrite(MOTOR2_SPEED, speed * 255 * globalSpeed); analogWrite(MOTOR4_SPEED, speed * 255 * globalSpeed);}

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

String detectColor() {                                                                //COLOR DETECTION (#CD)
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

void fieldDetect() {                                                                          //DROPOFF SYSTEM (#DO)

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

void TESTSENSORS() {                                                                                                      //DEBUG
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

    // Turn on the selected LEDs based on the desired color
    switch (color) {
        case 'R':
            digitalWrite(RED_PIN, HIGH);  // Red only
            break;
        case 'G':
            digitalWrite(GREEN_PIN, HIGH);  // Green only
            break;
        case 'B':
            digitalWrite(BLUE_PIN, HIGH);  // Blue only
            break;
        case 'C':
            // Cyan is a combination of Green and Blue
            digitalWrite(GREEN_PIN, HIGH);
            digitalWrite(BLUE_PIN, HIGH);
            break;
        case 'P':
            // Purple is a combination of Red and Blue
            digitalWrite(RED_PIN, HIGH);
            digitalWrite(BLUE_PIN, HIGH);
            break;
        case 'Y':
            // Yellow is a combination of Red and Green
            digitalWrite(RED_PIN, HIGH);
            digitalWrite(GREEN_PIN, HIGH);
            break;
        case 'W':
            // White is a combination of Red, Green, and Blue
            digitalWrite(RED_PIN, HIGH);
            digitalWrite(GREEN_PIN, HIGH);
            digitalWrite(BLUE_PIN, HIGH);
            break;
        default:
            // If the input is not recognized, all LEDs remain off
            break;
    }
}

void BLINKontrack(){
  unsigned long currentMillis = millis();

  if ((ledState == true) && (currentMillis - previousMillis >= onTime)) {
    // Turn off the LED after onTime
    setColor('X'); // Assuming 'X' turns off all LEDs
    ledState = false;
    previousMillis = currentMillis; // Remember the time it switched off
  } else if ((ledState == false) && (currentMillis - previousMillis >= offTime)) {
    // Turn on the LED after offTime
    setColor('G'); // Turn on Green LED
    ledState = true;
    previousMillis = currentMillis; // Remember the time it switched on
  }
}



float getSensor(String sensorID) {                                                                                                  //TOFS
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

