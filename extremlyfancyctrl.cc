#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_SPIDevice.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

#define motorPinL1 22
#define motorPinL2 24
#define motorPinR1 26
#define motorPinR2 28
#define motorSpeedL 8
#define motorSpeedR 9
#define echoPinFront 3
#define trigPinFront 2
#define echoPinLeft 5
#define trigPinLeft 4
#define echoPinRight 7
#define trigPinRight 6
#define ledpin 11
#define dropoffpin 30


int onelen = 250;
int burstlen = 500;
int caliback = 200;
int turnlen = 400;
int turnfwd = 100;

int betw = 250;

int colmaxval = 100;
int colminred = 50;
int distanceopen = 20;
  

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);


class UltraSonic{
  public:
    int returndistFront(){
      long duration;
      int distance; 
      digitalWrite(trigPinFront, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPinFront, HIGH);
      digitalWrite(trigPinFront, LOW);
      duration = pulseIn(echoPinFront, HIGH);
      if (duration == 0)
      {
        return 1000;
      }
      distance = duration * 0.034 / 2;
      return distance;
    }
  
    int returndistLeft(){
      long duration;
      int distance; 
      digitalWrite(trigPinLeft, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPinLeft, HIGH);
      digitalWrite(trigPinLeft, LOW);
      duration = pulseIn(echoPinLeft, HIGH);
      if (duration == 0)
      {
        return 1000;
      }
      distance = duration * 0.034 / 2;
      return distance;
    }
  
    int returndistRight(){
      long duration;
      int distance; 
      digitalWrite(trigPinRight, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPinRight, HIGH);
      digitalWrite(trigPinRight, LOW);
      duration = pulseIn(echoPinRight, HIGH);
      if (duration == 0)
      {
        return 1000;
      }
      distance = duration * 0.034 / 2;
      return distance;
    }
  
    void setupsensor(){  
      pinMode(trigPinFront, OUTPUT); 
      pinMode(echoPinFront, INPUT); 
      pinMode(trigPinLeft, OUTPUT); 
      pinMode(echoPinLeft, INPUT);
      pinMode(trigPinRight, OUTPUT); 
      pinMode(echoPinRight, INPUT); 
    }
};

class MotorCtrl{
  public:
    void setupmotors()
    {
      pinMode(motorPinL1, OUTPUT);
      pinMode(motorPinL2, OUTPUT);
      pinMode(motorPinR1, OUTPUT);
      pinMode(motorPinR2, OUTPUT);
      pinMode(motorSpeedL,OUTPUT);
      pinMode(motorSpeedR,OUTPUT);
      pinMode(dropoffpin,OUTPUT);
    }

    void setspeed(float speed){
      mL_SP(speed);
      mR_SP(speed);
    }

    void dropoff(){
      analogWrite(dropoffpin,HIGH);
      delay(50);
      analogWrite(dropoffpin,LOW);
    }
    
    void mL_SP(float speed){
      analogWrite(motorSpeedL, (255*speed));
    }
    
    void mR_SP(float speed){
      analogWrite(motorSpeedR, (255*speed));
    }

    void mL_BW(){
      digitalWrite(motorPinL1, HIGH);
      digitalWrite(motorPinL2, LOW);
    }

    void mL_FW(){
      digitalWrite(motorPinL1, LOW);
      digitalWrite(motorPinL2, HIGH);
    }

    void mR_FW(){
      digitalWrite(motorPinR1, LOW);
      digitalWrite(motorPinR2, HIGH);
    }
    
    void mR_BW(){
      digitalWrite(motorPinR1, HIGH);
      digitalWrite(motorPinR2, LOW);
    }

    void mL_OFF(){
      digitalWrite(motorPinR1, LOW);
      digitalWrite(motorPinR2, LOW);
    }

    void mR_OFF(){
      digitalWrite(motorPinL1, LOW);
      digitalWrite(motorPinL2, LOW);
    }
    void FW(){
      mL_FW();
      mR_FW();
    }
    void BW(){
      mL_BW();
      mR_BW();
    }
    void LEFT(){
      mL_BW();
      mR_FW();
    }
    void RIGHT(){
      mL_FW();
      mR_BW();
    }
    void STOP(){
      mL_OFF();
      mR_OFF();
    }
};

class Color{
  public:
  
    uint16_t r, g, b, c, colorTemp, lux;
    
    void setupcolor(){
      if(tcs.begin()){
        Serial.println("COLOR INIT");
      }
      else{
         Serial.println("ERROR COLORS");
      }
    }
    void ReturnColor(){
      
      r = 0;
      g = 0;
      b = 0;
          
    
      tcs.getRawData(&r, &g, &b, &c);
      // colorTemp = tcs.calculateColorTemperature(r, g, b);
      colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
      lux = tcs.calculateLux(r, g, b);

    }
};
    



void ledsend(){
  int i = 0;
  int len = 100;
  while (i < 6000){
    
    digitalWrite(ledpin, HIGH);
    delay(len);
    digitalWrite(ledpin, LOW);
    delay(len);
    i+= len*2;
  }
  
}



void setup() {
  Serial.begin(9600);
  pinMode(ledpin, OUTPUT); 
  MotorCtrl Motors;
  Motors.setupmotors();
  Motors.setspeed(0.4);
  UltraSonic Sonic;
  Sonic.setupsensor();
  Color ColorSensor;
  ColorSensor.setupcolor();
  
  Serial.println("setup complete!");

}



bool checkred(){
  Color ColorSensor;
  ColorSensor.ReturnColor();
  MotorCtrl Motors;
  Serial.println("COLOR FOR RED:");
  int colr = ColorSensor.r;
  int colg = ColorSensor.g;
  int colb = ColorSensor.b;
  
  Serial.println(colr);
  Serial.println(colg);
  Serial.println(colb);
  if ((colr - colminred >= colg) && (colr - colminred >= colb)){
    ledsend();
    Motors.dropoff();
    return true;
  }
  return false;
}
bool checkblack(){
  Color ColorSensor;
  ColorSensor.ReturnColor();
  MotorCtrl Motors;
  Serial.println("COLOR FOR BLACK:");
  int colr = ColorSensor.r;
  int colg = ColorSensor.g;
  int colb = ColorSensor.b;
  Serial.println(colr);
  Serial.println(colg);
  Serial.println(colb);
  if ((colr <= colmaxval) && (colg <= colmaxval) && (colb <= colmaxval)){
    
    Serial.println("BLACKKKK");
    Motors.LEFT();
    delay(turnlen);
    Motors.STOP();
    delay(betw);
    Motors.FW();
    delay(burstlen);
    return true;
  }
  return false;
}


int getnextstep(bool left,bool front,bool right){
  if (right)
  {
    return 3;
  }
  if (front){
    return 2;
  }
  if (left){
    return 1;
  }
  
  
  return 0;
}
void step(){

  MotorCtrl Motors;
  UltraSonic Sonic;
  Motors.STOP();
  Serial.println("sonic...");
  delay (betw);
  float Leftval = Sonic.returndistLeft();
  delay (betw);
  float Frontval = Sonic.returndistFront();
  delay (betw);
  float Rightval = Sonic.returndistRight();
  delay (betw);
  Serial.println("...sonicend");
  
  Serial.println("sensors LFR:");
  Serial.println(Leftval);
  Serial.println(Frontval);
  Serial.println(Rightval);
  Serial.println("next sequence:");
  int next = getnextstep(Leftval > distanceopen,Frontval > distanceopen,Rightval > distanceopen);
  Serial.println(next);
  
  Serial.println("checking for colors...");
  checkblack();
  checkred();
  delay(betw);

  if (Frontval < distanceopen){
    Motors.FW();
    delay(burstlen);
    Motors.STOP();
    delay(betw);
    Motors.BW();
    delay(caliback);
    Motors.STOP();
    delay(betw);

    float Leftval = Sonic.returndistLeft();
    delay (betw);
    float Frontval = Sonic.returndistFront();
    delay (betw);
    float Rightval = Sonic.returndistRight();
    delay (betw);
    
    int next = getnextstep(Leftval > distanceopen,Frontval > distanceopen,Rightval > distanceopen);
  }
  

  if (next == 2){
    Motors.FW();
    delay(onelen);
  }
  


  if (next == 1 || next == 3){
    Motors.FW();
    delay(turnfwd);
    Motors.STOP();
    delay(betw);
  }
  if (next == 1){
    Motors.LEFT();
    delay(turnlen);
    Motors.STOP();
    delay(betw);
    Motors.FW();
    delay(burstlen);
  }
  if (next == 3){
    Motors.RIGHT();
    delay(turnlen);
    Motors.STOP();
    delay(betw);
    Motors.FW();
    delay(burstlen);
  }
  if(next == 0){
    Motors.RIGHT();
    delay(turnlen);
    Motors.STOP();
    delay(betw);
  }





  //if (next == 0){
  //  Motors.RIGHT();
 //   delay(turnlen*2);
  //  Motors.STOP();
  //  delay(betw);
    //Motors.FW();
    //delay(burstlen);
  //}

}

void loop(){
  MotorCtrl Motors;
  UltraSonic Sonic;
  //checkred();
  step();
  //Motors.mR_BW();
  //Motors.mL_BW();
  delay(100);
}
