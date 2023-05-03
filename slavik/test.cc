#define motorPinL1 2
#define motorPinL2 3
#define motorPinR1 4
#define motorPinR2 5
#define motorSpeedL 9
#define motorSpeedR 10

//1665+14 = js

void setuppins()
{
  pinMode(motorPinL1, OUTPUT);
  pinMode(motorPinL2, OUTPUT);
  pinMode(motorPinR1, OUTPUT);
  pinMode(motorPinR2, OUTPUT);
  //pinMode(motorSpeedL,OUTPUT);
  //pinMode(motorSpeedR,OUTPUT);
}

void Test()
{
  digitalWrite(motorPinL1, LOW);
  digitalWrite(motorPinL2, HIGH);
  digitalWrite(motorPinR1, LOW);
  digitalWrite(motorPinR2, HIGH);
}

void setup(){
  Serial.begin(9600);
  setuppins();
}

void loop(){
  
  Test();
}
