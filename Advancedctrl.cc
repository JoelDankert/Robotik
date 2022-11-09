#define motorPinL1 2
#define motorPinL2 3
#define motorPinR1 4
#define motorPinR2 5
#define motorSpeedL 9
#define motorSpeedR 10
#define echoPin 13
#define trigPin 12

class UltraSonic{
  public:
    int returndist(){
      long duration;
      int distance; 
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      digitalWrite(trigPin, LOW);
      duration = pulseIn(echoPin, HIGH);
      distance = duration * 0.034 / 2;
      return distance;
    }
    void setupsensor(){  
      pinMode(trigPin, OUTPUT); 
      pinMode(echoPin, INPUT); 
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

    void mR_BW(){
      digitalWrite(motorPinL1, LOW);
      digitalWrite(motorPinL2, HIGH);
    }

    void mL_BW(){
      digitalWrite(motorPinR1, HIGH);
      digitalWrite(motorPinR2, LOW);
    }

    void mR_FW(){
      digitalWrite(motorPinL1, HIGH);
      digitalWrite(motorPinL2, LOW);
    }

    void mL_FW(){
      digitalWrite(motorPinR1, LOW);
      digitalWrite(motorPinR2, HIGH);
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
  Motors.setspeed(0.2);
  UltraSonic Sonic;
  Sonic.setupsensor();
  
  Serial.println("setup complete!");

  for (float i = 10; i>0;i--){
    Serial.println("KUPFER");
    delay(100);
    Serial.println("starting in "+String(i/10)+"sec");
  }
}

void loop()
{
  MotorCtrl Motors;
  UltraSonic Sonic;
  goinrange(25,5);
  Motors.LEFT();
  delay(2500);
}
