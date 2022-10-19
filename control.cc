#define motorPinL1 2
#define motorPinL2 3
#define motorPinR1 4
#define motorPinR2 5
#define motorSpeedL 9
#define motorSpeedR 10

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
      
      analogWrite(9, (200*speed)); //ENA pin
      analogWrite(10, (200*speed)); //ENB pin 
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
    void FW(){
      mL_FW();
      mR_FW();
    }
    void BW(){
      mL_BW();
      mR_BW();
    }
    void Left(){
      mL_FW();
      mR_BW();
    }
    void Right(){
      mL_BW();
      mR_FW();
    }
};

void setup() {
  
  MotorCtrl Motors;
  Serial.begin(9600);
  Motors.setupmotors();
  Serial.println("setup complete!");

  for (float i = 10; i>0;i--){
    Serial.println("KUPFER");
    delay(100);
    Serial.println("starting in "+String(i/10)+"sec");
  }
}

int dlay = 5000;

void loop(){
  MotorCtrl Motors;
  Motors.FW();
  delay(1000);
  Motors.setspeed(0.1);
  delay(1000);
  Motors.setspeed(0.5);
  delay(1000);
  Motors.setspeed(1);
}
