#define motorPinL1 2
#define motorPinL2 3
#define motorPinR1 4
#define motorPinR2 5
#define motorSpeedL 9
#define motorSpeedR 10
#define echoPinFront 13
#define trigPinFront 12
#define echoPinLeft 13
#define trigPinLeft 12
#define echoPinRight 13
#define trigPinRight 12

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

void goinrange(int des, int buff){
  int tostay = 0;
  while (1 == 1)
  {
    delay(10);
    MotorCtrl Motors;
    UltraSonic Sonic;
    int desired = des;
    int maxovershoot = buff;
  
    float dist = Sonic.returndist();
    Serial.println(String(dist));
    if (dist > desired + maxovershoot)
    {
      Motors.FW();
      tostay = 0;
    }
    else if (dist < desired - maxovershoot)
    {
      Motors.BW();
      tostay = 0;
    }
    else
    {
      Motors.STOP();
      tostay += 1;
    }
    
    if (tostay >= 50)
    {
      return;
    }
  }

}

void setup() {
  Serial.begin(9600);
  MotorCtrl Motors;
  Motors.setupmotors();
  Motors.setspeed(0.5);
  UltraSonic Sonic;
  Sonic.setupsensor();
  
  Serial.println("setup complete!");

}


int getnextstep(left,front,right){
  if (left){
    return 1
  }
  if (front){
    return 2
  }
  if (right)
  {
    return 3
  }
  return 0
}

void step(){
  MotorCtrl Motors;
  UltraSonic Sonic;
  float Leftval = Sonic.returndistLeft
  float Frontval = Sonic.returndistFront
  float Rightval = Sonic.returndistRight
  int distanceopen = 5
  int next = getnextstep(Leftval > distanceopen,Frontval > distanceopen,Rightval > distanceopen)
  int onelen = 1
  if (next = 0){
    Motors.LEFT()
    delay(onelen*2)
    Motors.STOP()
  }
  else if (next = 1){
    Motors.LEFT()
    delay(onelen)
    Motors.STOP()
  }
  else if (next = 2){
    Motors.FW()
    delay(onelen)
    Motors.STOP()
  }
  else if (next = 3){
    Motors.RIGHT()
    delay(onelen)
    Motors.STOP()
  }
}

void loop()
{
  MotorCtrl Motors;
  UltraSonic Sonic;
  goinrange(25,3);
  delay(1000);
}
