
#define echoPin 13
#define trigPin 12
#define motorPin 2

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

void startmotor(){
  digitalWrite(motorPin, HIGH);
}

void stopmotor(){
  digitalWrite(motorPin, LOW);
}

void setup() {
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 
  pinMode(motorPin, OUTPUT); 
  Serial.begin(9600);
}
void loop() {
  int dist = returndist();
  Serial.println("current dist: "+String(dist)+"cm");
  if (dist > 10){
    startmotor();
    Serial.println("KUPFER");
  }
  else{
    stopmotor();
  }
}
