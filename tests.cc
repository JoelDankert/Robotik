
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

void setup() {
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 
  Serial.begin(9600);
  Serial.println("Ultrasonic Sensor HC-SR04 Test"); 
  Serial.println("with Arduino UNO R3");
}
void loop() {
  int dist = returndist();

 
}
