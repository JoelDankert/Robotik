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
      pinMode(motorSpeedL,OUTPUT);
    }

    void setspeed(float speed){
      analogWrite(9, (255*speed)); //ENA pin
      analogWrite(10, (255*speed)); //ENB pin 
    }

    void mL_BW(){
      digitalWrite(motorPinL1, HIGH);
      digitalWrite(motorPinR2, LOW);
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
      mL_FW();
      mR_BW();
    }
    void RIGHT(){
      mL_BW();
      mR_FW();
    }
    void STOP(){
      mL_OFF();
      mR_OFF();
    }
};


void setup() {
  Serial.begin(9600);
  MotorCtrl Motors;
  Motors.setupmotors();
  Motors.setspeed(0.5);
  UltraSonic Sonic;
  Sonic.setupsensor();
  
  Serial.println("setup complete!");

}


int getnextstep(bool left,bool front,bool right){
  if (left){
    return 1;
  }
  if (front){
    return 2;
  }
  if (right)
  {
    return 3;
  }
  return 0;
}

void step(){
  MotorCtrl Motors;
  UltraSonic Sonic;
  float Leftval = Sonic.returndistLeft();
  float Frontval = Sonic.returndistFront();
  float Rightval = Sonic.returndistRight();
  int distanceopen = 5;
  int next = getnextstep(Leftval > distanceopen,Frontval > distanceopen,Rightval > distanceopen);
  int onelen = 0.2;
  if (next = 0){
    Motors.LEFT();
    delay(onelen*2);
    Motors.STOP();
  }
  else if (next = 1){
    Motors.LEFT();
    delay(onelen);
    Motors.STOP();
  }
  else if (next = 2){
    Motors.FW();
    delay(onelen);
    Motors.STOP();
  }
  else if (next = 3){
    Motors.RIGHT();
    delay(onelen);
    Motors.STOP();
  }
}

void loop()
{
  MotorCtrl Motors;
  UltraSonic Sonic;
  step();
  delay(5000);
}
