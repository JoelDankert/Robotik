#define motorPinL1 2
#define motorPinL2 3
#define motorPinR1 4
#define motorPinR2 5

class MotorCtrl{
  public:
    void setupmotors()
    {
      pinMode(motorPinL1, OUTPUT);
      pinMode(motorPinL2, OUTPUT);
      pinMode(motorPinR1, OUTPUT);
      pinMode(motorPinR2, OUTPUT);
    }

    void mR_FW(){
      digitalWrite(motorPinL1, LOW);
      digitalWrite(motorPinL2, HIGH);
    }

    void mL_FW(){
      digitalWrite(motorPinR1, HIGH);
      digitalWrite(motorPinR2, LOW);
    }

    void mR_BW(){
      digitalWrite(motorPinL1, HIGH);
      digitalWrite(motorPinL2, LOW);
    }

    void mL_BW(){
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
}

class ShortCtrl{
  public:
    void Forward(){
      MotorCtrl.mL_FW();
      MotorCtrl.mR_FW();
    }
    void Backward(){
      MotorCtrl.mL_BW();
    	MotorCtrl.mR_BW();
    }
    void LeftFull(){
      MotorCtrl.mL_BW();
      MotorCtrl.mR_FW();
    }
    void RightFull{
       MotorCtrl.mL_FW();
       MotorCtrl.mR_BW();
    }
    void Off{
      MotorCtrl.mL_OFF();
      MotorCtrl.mR_OFF();
    }
    

}


void setup() {
  Serial.begin(9600);
  setupmotors();
  Serial.println("setup complete!");

  for (float i = 50; i>0;i--){
    Serial.println("KUPFER");
    delay(100);
    Serial.println("starting in "+String(i/10)+"sec");
  }
}

int dlay = 5000;

void loop(){
  Serial.println("LOOP START");
  MotorCtrl.mL_FW();
  MotorCtrl.mR_FW();
  Serial.println("fw");
  delay(dlay);
  MotorCtrl.mL_BW();
  MotorCtrl.mR_BW();
  Serial.println("bw");
  delay(dlay);
  MotorCtrl.mL_FW();
  MotorCtrl.mR_BW();
  Serial.println("right");
  delay(dlay);
  MotorCtrl.mL_BW();
  MotorCtrl.mR_FW();
  Serial.println("left");
  delay(dlay);
  MotorCtrl.mL_OFF();
  MotorCtrl.mR_FW();
  Serial.println("left off");
  delay(dlay);
  MotorCtrl.mL_FW();
  MotorCtrl.mR_OFF();
  Serial.println("right off");
  delay(dlay);
  MotorCtrl.mL_OFF();
  MotorCtrl.mR_OFF();
  Serial.println("full off");
  delay(dlay);
  
  Serial.println("LOOP END");
}
