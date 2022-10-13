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
};

void setup() {
  
  MotorCtrl Motors;
  Serial.begin(9600);
  Motors.setupmotors();
  Serial.println("setup complete!");

  for (float i = 50; i>0;i--){
    Serial.println("KUPFER");
    delay(100);
    Serial.println("starting in "+String(i/10)+"sec");
  }
}

int dlay = 5000;

void loop(){
  MotorCtrl Motors;
  Serial.println("LOOP START");
  Motors.mL_FW();
  Motors.mR_FW();
  Serial.println("fw");
  delay(dlay);
  Motors.mL_BW();
  Motors.mR_BW();
  Serial.println("bw");
  delay(dlay);
  Motors.mL_FW();
  Motors.mR_BW();
  Serial.println("right");
  delay(dlay);
  Motors.mL_BW();
  Motors.mR_FW();
  Serial.println("left");
  delay(dlay);
  Motors.mL_OFF();
  Motors.mR_FW();
  Serial.println("left off");
  delay(dlay);
  Motors.mL_FW();
  Motors.mR_OFF();
  Serial.println("right off");
  delay(dlay);
  Motors.mL_OFF();
  Motors.mR_OFF();
  Serial.println("full off");
  delay(dlay);
  
  Serial.println("LOOP END");
}
