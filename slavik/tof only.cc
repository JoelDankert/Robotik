#include <Wire.h>
#include <VL53L0X.h>

// Create VL53L0X objects for each sensor
VL53L0X F;
VL53L0X RF;
VL53L0X RB;
VL53L0X B;

// Define the pins connected to the XSHUT (shutdown) pin of each sensor
const int XSHUT_pin_RF = 3; // Example pin for Right Front sensor
const int XSHUT_pin_RB = 2; // Example pin for Right Back sensor
const int XSHUT_pin_F = 1;  // Example pin for Front sensor
const int XSHUT_pin_B = 4;  // Example pin for Back sensor

// Define unique I2C addresses for the sensors
const uint8_t addressRF = 0x30; // Right Front
const uint8_t addressRB = 0x31; // Right Back
const uint8_t addressF = 0x32;  // Front
const uint8_t addressB = 0x33;  // Back

bool wakeSensorAndSetAddress(int pin, VL53L0X& sensor, uint8_t newAddress) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delay(10);
  digitalWrite(pin, HIGH);
  delay(10);
  if (!sensor.init()) {
    Serial.println("Sensor init failed");
    return false;
  }
  sensor.setAddress(newAddress);
  Serial.print("Sensor at pin ");
  Serial.print(pin);
  Serial.println(" setup");
  return true;
}


void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Wake up each sensor one by one, assign a new address, and initialize
  wakeSensorAndSetAddress(XSHUT_pin_F, F, addressF);
  wakeSensorAndSetAddress(XSHUT_pin_RF, RF, addressRF);
  wakeSensorAndSetAddress(XSHUT_pin_RB, RB, addressRB);
  wakeSensorAndSetAddress(XSHUT_pin_B, B, addressB);
}

int getSensorData(String sensorID) {
  if (sensorID == "F") {
    return F.readRangeSingleMillimeters();
  } else if (sensorID == "RF") {
    return RF.readRangeSingleMillimeters();
  } else if (sensorID == "RB") {
    return RB.readRangeSingleMillimeters();
  } else if (sensorID == "B") {
    return B.readRangeSingleMillimeters();
  } else {
    Serial.println("Invalid sensor ID");
    return -1; // Indicate an error
  }
}

void loop() {
  // Example usage
  Serial.print("F Distance: ");
  Serial.println(getSensorData("F"));

  Serial.print("RF Distance: ");
  Serial.println(getSensorData("RF"));

  Serial.print("RB Distance: ");
  Serial.println(getSensorData("RB"));

  Serial.print("B Distance: ");
  Serial.println(getSensorData("B"));

  delay(1000); // Delay between readings
}
