//MEGA
#include <Adafruit_TCS34725.h>
#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>

// TOF Sensor objects
VL53L0X FF;
VL53L0X RF;
VL53L0X RB;
VL53L0X BB;

// TOF Sensor pins
const int XSHUT_pin_RF = 25;  // Example pin for Right Front sensor
const int XSHUT_pin_RB = 26;  // Example pin for Right Back sensor
const int XSHUT_pin_F = 24;   // Example pin for Front sensor
const int XSHUT_pin_B = 27;   // Example pin for Back sensor

// TOF Sensor addresses
const uint8_t addressRF = 0x30;  // Right Front
const uint8_t addressRB = 0x31;  // Right Back
const uint8_t addressF = 0x32;   // Front
const uint8_t addressB = 0x33;   // Back

// TOF Sensor offsets
const int Foffset = -1;
const int RBoffset = -1;
const int RFoffset = -1;
const int Boffset = -1;

// Motor pins
#define MOTOR1_DIR 4
#define MOTOR1_SPEED 3
#define MOTOR2_DIR 12
#define MOTOR2_SPEED 11
#define MOTOR3_DIR 8
#define MOTOR3_SPEED 5
#define MOTOR4_DIR 7
#define MOTOR4_SPEED 6

// Color detection pins
#define redPin 29    
#define blackPin 30  
#define resetPin 31  
#define greenPin 32  

#define tickPin 42

// Servo pin
#define servopin 22

// LED pins
#define pinLED 40
#define RED_PIN 51
#define GREEN_PIN 52
#define BLUE_PIN 53

// Reset function
void (*resetFunc)(void) = 0;

// Distance thresholds
int rightWallDistanceMax = 14;
int frontWallDistanceGoal = 12;
int frontWallDistanceMin = 8;
int tryWallDistanceGoal = 20;
int rightWallDistanceGoal = 7;

// Speed and time constants
float globalSpeed = 1;
int fieldSize = 30;
float timeperangle = 6;
int rightturncancel = 13;
const float Frightturn = 0.1;
const float Fleftturnspeed = 1;

// Color detection variables
int lastred = 0;
int reddelay = 4000;
bool hasclearedred = true;
bool hasclearedgreen = false;

// State variablesf
int state = 0;

// Error detection variables
float EDcurrentchange = 30;
float EDresetvalue = 30;
int EDlast = 0;
float EDcap = 3;
float EDchangeslow = 10;
int fatalerrorcount = 0;
int fatalerrorreset = 100;
float lastFront = 0;
float frontmax = 5;
bool tickState = false;
int dashStrength = 600;
int dashStrengthBack = 1000;
int dashFrequency = 1200;
float dashSpeed = 0.7;
float dashSpeedBack = 0.5;
long lastExecutedDash = 0;  // Time when dashTick was last executed
int minfrontdistdash = 23;

// Debug flags
bool debug = false;
bool nocolor = false;
bool nodash = false;
bool noblack = false;

// Blink variables
unsigned long previousMillis = 0;  // will store last time LED was updated
const long onTime = 25;            // milliseconds of on-time
const long offTime = 100;          // milliseconds of off-time
bool ledState = false;

// Servo object
Servo dropoff;


void setup() {  //SETUP


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
  pinMode(greenPin, INPUT);
  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, HIGH);

  pinMode(tickPin, OUTPUT);





  dropoff.attach(servopin);  

  dropoff.write(180);
  delay(400);
  toggleTick();
  delay(400);
  toggleTick();
  delay(400);
  toggleTick();
  delay(400);
  toggleTick();
  delay(400);
  toggleTick();
  delay(400);
  dropoff.write(90);       

  digitalWrite(pinLED, LOW);

  delay(500);


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


  resetSignal();
  delay(100);


}

void loop() {

  //spin(90);

  //moveBackward(0.5);
  //TESTSENSORS();
  //Serial.print(detectColor());
  //delay(3000);
  //ResetSensors();
  
  MAIN();


  //servodrop();

  delay(1000);
}

void MAIN() {
  while (true) {


    
    delay(10);
    toggleTick();
    
    if (debug){
      TESTSENSORS();
    }

    if (state == 0 || state == -1) {  //RESETLED
      setColor('X');
    }
    state = -1;



   if (trydetcol()>0){
    continue;
   }



    int spd = 1;  //LEFT TURNS (#LT)
    Serial.println("sensor grab");
    int front = getSensor("FF");
    Serial.println("F check");
    float rightF = getSensor("RF");
    Serial.println("RF check");
    float rightB = getSensor("RB");
    Serial.println("RB check");
    Serial.println("sensor grab finished");

    if (errordetecttick(front)) {  //ERRORDETECTTICK (#EDT)
      Serial.println("err det");
      moveBackward(1);
      setColor('W');
      delay(500);
      motorsOff();
    }

    int i = 0;
    if( front < frontWallDistanceMin){
      Serial.println("                              left turn: ");
      Serial.println(">");
      bool skip = false;
      lastFront = front;
      while (front < frontWallDistanceGoal && i < 20) {
        
        toggleTick();
        skip = false;
        i++;
        turnLeft(Fleftturnspeed);
        if (state == -1) {
          state = 3;
          setColor('B');
        }
        Serial.print(".");
        trydetcol();
        if (front > lastFront + frontmax && lastFront < 30) {  //Suboptimal Left Turn Quantification Compensator
          Serial.print("!C!");
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
        if (skip) {
          Serial.print("//");
          continue;
        }




        if (errordetecttick(front)) {
          Serial.print("!ED!");
          moveBackward(1);
          delay(500);
          motorsOff();
        }
      }
      
      Serial.println(".");

      if (state != -1) {
        continue;
      }
    }
    




    //RIGHT TURNS (#RT)

    if (rightF > rightWallDistanceMax) {
      Serial.println("                              right turn: ");
      Serial.print(rightF);
      if(!nodash){
        dashTick(front);
      }
      
      if (state == -1) {
        state = 2;
        setColor('B');
      }
      if (front < rightturncancel) {

      } else {
        moveForward(1);
        setMotorSpeedR(Frightturn);
      }
    }
    if (state != -1) {
      continue;
    }

    //Exponential Wall-Alignment Righting Mechanism EWARM (#WC)


    float distadj = (rightF - rightWallDistanceGoal) * 1;

    float diff = rightF - rightB + distadj;
    if (rightF <= tryWallDistanceGoal && rightB <= tryWallDistanceGoal) {
      
      Serial.println("                              alligning... ");
      moveForward(1);
      if (state == -1) {
        state = 1;
        setColor('G');
      }

      float compamount = max(min((pow(abs(diff), 2.5) * 0.06), 1), 0);
      float spdC = -1 * compamount + 1;
      if (compamount < 0.3) {
        BLINKontrack();
      }
      Serial.println("adjustment needed: ");
      Serial.print(compamount);

      if (diff > 0) {
        setMotorSpeedR(spdC);
      } else {
        setMotorSpeedL(spdC);
      }
    }
    if (state != -1) {
      continue;
    }





    if (state == -1) {
      moveForward(1);
    }
  }
}


bool errordetecttick(int front) {  //Progressive Stasis Error Detection PSED (#ED)



  float absolutechange = abs(front - EDlast);
  EDcurrentchange = (EDcurrentchange * EDchangeslow + absolutechange) / (EDchangeslow + 1);


  if (EDcurrentchange > EDresetvalue) {
    EDcurrentchange = EDresetvalue;
  }
  if (EDcurrentchange <= EDcap) {
    EDcurrentchange = EDresetvalue;
    return true;
  }

  EDlast = front;


  return false;
}


void setMotorSpeed(float speed) {  //MOTORFUNCS
  int pwmValue = speed * 255 * globalSpeed;
  analogWrite(MOTOR1_SPEED, pwmValue);
  analogWrite(MOTOR2_SPEED, pwmValue);
  analogWrite(MOTOR3_SPEED, pwmValue);
  analogWrite(MOTOR4_SPEED, pwmValue);
}

void setMotorSpeedL(float speed) {
  analogWrite(MOTOR1_SPEED, speed * 255 * globalSpeed);
  analogWrite(MOTOR3_SPEED, speed * 255 * globalSpeed);
}

void setMotorSpeedR(float speed) {
  analogWrite(MOTOR2_SPEED, speed * 255 * globalSpeed);
  analogWrite(MOTOR4_SPEED, speed * 255 * globalSpeed);
}

void moveForward(float spd) {
  if (spd != -1) {
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

String detectColor() {  //COLOR DETECTION (#CD)
  //return "none";

  bool redSignal = digitalRead(redPin) == HIGH;
  bool blackSignal = digitalRead(blackPin) == HIGH;
  bool greenSignal = digitalRead(greenPin) == HIGH;


  if(millis() > lastred+reddelay){
      if(!hasclearedred){
        Serial.println("                              red RESETTED");
        hasclearedred = true;
        resetSignal();
        return "none";
      }
    }

  if (millis() > 10000 && !hasclearedgreen){
    Serial.println("                              green RESETTED");
    hasclearedgreen = true;
    resetSignal();
    return "none";
  }
  
  if (redSignal && hasclearedred) {
    if(millis() > lastred+reddelay && hasclearedred){
      return "Red";
    }
  }
  if (greenSignal) {
    if(millis() > 10000){
      return "Green";
    }
  }
  if (blackSignal) {
    return "Black";
  }
  return "none";  // No color detected
}

int trydetcol(){
   String det = "none";
    if (!nocolor){
      det = detectColor();
    }
    if (det == "Red") {
      Serial.println("                              red detected");
      fieldDetect();
      lastred = millis();
      hasclearedred = false;
      resetSignal();
      return 1;
    }
    if (det == "Black" && !noblack) {
      Serial.println("                              black detected");
      setColor('B');
      moveBackward(1);
      delay(400);
      turnLeft(1);
      toggleTick();
      delay(600);
      toggleTick();
      motorsOff();
      resetSignal();
      return 2;
    }
    if (det == "Green") {
      Serial.println("                              exit bonus");
      setColor('C');
      moveForward(1);
      delay(500);
      motorsOff();
      for (int x = 0; x<13000; x+=100){
        delay(100);
        toggleTick();
      }
      ResetSensors();
      
    }
    return 0;
}

void resetSignal() {
  motorsOff();
  digitalWrite(resetPin, LOW);   
  delay(200);                    
  digitalWrite(resetPin, HIGH);  
  delay(100);
}
void toggleTick(){
  tickState = !tickState; 
  digitalWrite(tickPin, tickState); 

}


void dashTick(int front) {

  Serial.println("dashtick > ");
  Serial.print(millis()-lastExecutedDash);
  if (millis() - lastExecutedDash < dashFrequency) {
    return;  
  }
  if (front < minfrontdistdash){
    return;
  }

  
  unsigned long elapsed = 0;  

  motorsOff();
  delay(50);
  moveForward(dashSpeed);

  while (elapsed < dashStrength) {
    int out = trydetcol();               
    toggleTick();
    delay(100);                 
    elapsed += 100;        
    if (out == 2){
      return;     
    }
    if (out == 1){
      moveForward(dashSpeed);
    }
  }

  motorsOff();
  delay(50);
  moveBackward(dashSpeedBack);

  elapsed = 0;
  
  while (elapsed < dashStrengthBack) {
    int out = trydetcol();               
    toggleTick();
    delay(100);                 
    elapsed += 100;        
    if (out == 2){
      return;     
    }
    if (out == 1){
      moveBackward(dashSpeedBack);
    }
  }

     
  motorsOff();
  delay(50);
  lastExecutedDash = millis();

}



void fieldDetect() {  //DROPOFF SYSTEM (#DO)
  motorsOff();
  setColor('R');
  for (int i = 0; i < 22; i++) {
    toggleTick();
    delay(250);
  }

  setColor('X');

  dropoff.write(0);
  delay(300);
  toggleTick();
  delay(300);
  toggleTick();
  delay(300);
  toggleTick();
  delay(300);
  toggleTick();
  delay(300);
  toggleTick();
  delay(300);
  toggleTick();
  delay(300);
  dropoff.write(90);
  toggleTick();
  delay(50);
  dropoff.write(180);
  delay(300);
  toggleTick();
  delay(300);
  toggleTick();
  delay(300);
  toggleTick();
  delay(300);
  toggleTick();
  delay(300);
  toggleTick();
  delay(300);
  toggleTick();
  delay(300);
  toggleTick();
  delay(300);
  toggleTick();
  delay(300);
  dropoff.write(90);
  toggleTick();
}

void TESTSENSORS() {  //DEBUG
  Serial.println("sensors: ");
  int distanceFront = getSensor("FF");
  Serial.println("F check");
  int distanceRightFront = getSensor("RF");
  Serial.println("RF check");
  int distanceRightBack = getSensor("RB");
  Serial.println("RF check");

  // Write the distances to the serial port
  Serial.print("Front: ");
  Serial.print(distanceFront);
  Serial.print(" cm, Right Front: ");
  Serial.print(distanceRightFront);
  Serial.print(" cm, Right Back: ");
  Serial.print(distanceRightBack);
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
      
      digitalWrite(GREEN_PIN, HIGH);
      digitalWrite(BLUE_PIN, HIGH);
      break;
    case 'P':
      
      digitalWrite(RED_PIN, HIGH);
      digitalWrite(BLUE_PIN, HIGH);
      break;
    case 'Y':
      
      digitalWrite(RED_PIN, HIGH);
      digitalWrite(GREEN_PIN, HIGH);
      break;
    case 'W':
      
      digitalWrite(RED_PIN, HIGH);
      digitalWrite(GREEN_PIN, HIGH);
      digitalWrite(BLUE_PIN, HIGH);
      break;
    default:
      // If the input is not recognized, all LEDs remain off
      break;
  }
}

void BLINKontrack() {
  unsigned long currentMillis = millis();

  if ((ledState == true) && (currentMillis - previousMillis >= onTime)) {
    
    setColor('X');  
    ledState = false;
    previousMillis = currentMillis;
  } else if ((ledState == false) && (currentMillis - previousMillis >= offTime)) {
    
    setColor('G');  
    ledState = true;
    previousMillis = currentMillis;  // Remember the time it switched on
  }
}



float getSensor(String sensorID) {  //TOFS
  fatalerrorcount -= 1;
  if (fatalerrorcount > fatalerrorreset) {
    ResetSensors();
  }


  if (sensorID == "FF") {
    int v = FF.readRangeSingleMillimeters();
    if (v <= 1) {
      fatalerrorcount += 10;
      return 1000;
    }


    float returnval = v / 10 + Foffset;
    return returnval;



  } else if (sensorID == "RF") {
    int v = RF.readRangeSingleMillimeters();
    if (v <= 1) {
      fatalerrorcount += 10;
      return 1000;
    }

    float returnval = v / 10 + RFoffset;
    return returnval;

  } else if (sensorID == "RB") {
    int v = RB.readRangeSingleMillimeters();
    if (v <= 1) {
      fatalerrorcount += 10;
      return 1000;
    }


    float returnval = v / 10 + RBoffset;
    return returnval;

  } else if (sensorID == "BB") {
    int v = BB.readRangeSingleMillimeters();
    if (v <= 1) {
      //fatalerrorcount+=10;
      return 1000;
    }


    float returnval = v / 10 + Boffset;
    return returnval;


  } else {
    Serial.println("Invalid sensor ID");
    return 1000;  // Indicate an error
  }
}

void ResetSensors() {
  setColor('X');
  resetFunc();
}

void initializeSensors() {
  delay(50);
  digitalWrite(XSHUT_pin_F, HIGH);
  delay(50);
  
  Serial.println("sensor init");
  if (!FF.init(true)) {
    Serial.println("F Sensor init failed");
  }
  FF.setAddress(addressF);
  RB.setTimeout(500);

  delay(50);
  digitalWrite(XSHUT_pin_RF, HIGH);
  delay(50);
  if (!RF.init()) {
    Serial.println("RF Sensor init failed");
  }
  RF.setAddress(addressRF);
  RB.setTimeout(500);


  delay(50);
  digitalWrite(XSHUT_pin_RB, HIGH);
  delay(50);
  if (!RB.init()) {
    Serial.println("RB Sensor init failed");
  }
  RB.setAddress(addressRB);
  RB.setTimeout(500);


  delay(50);
  digitalWrite(XSHUT_pin_B, HIGH);
  delay(50);
  if (!BB.init()) {
    Serial.println("B Sensor init failed");
  }
  BB.setAddress(addressB);
  RB.setTimeout(500);
  
  Serial.println("sensor init completed");
}
