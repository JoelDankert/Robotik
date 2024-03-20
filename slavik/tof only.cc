#include <Wire.h>
#include <VL53L0X.h>

// Create a VL53L0X object
VL53L0X sensor;

// Define the pins connected to the XSHUT (shutdown) pin of each sensor
const int XSHUT_pin_RF = 2; // Example pin for Right Front sensor
const int XSHUT_pin_RB = 3; // Example pin for Right Back sensor
const int XSHUT_pin_F = 4;  // Example pin for Front sensor
const int XSHUT_pin_B = 5;  // Example pin for Back sensor

// Define unique I2C addresses for the sensors
const uint8_t addressRF = 0x30; // Right Front
const uint8_t addressRB = 0x31; // Right Back
const uint8_t addressF = 0x32;  // Front
const uint8_t addressB = 0x33;  // Back

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(XSHUT_pin_RF, OUTPUT);
  digitalWrite(XSHUT_pin_RF, LOW);
  pinMode(XSHUT_pin_RB, OUTPUT);
  digitalWrite(XSHUT_pin_RB, LOW);
  pinMode(XSHUT_pin_F, OUTPUT);
  digitalWrite(XSHUT_pin_F, LOW);
  pinMode(XSHUT_pin_B, OUTPUT);
  digitalWrite(XSHUT_pin_B, LOW);
  delay(10); // Ensure all sensors are off

  // Wake up each sensor one by one, assign a new address, and initialize
  wakeSensorAndSetAddress(XSHUT_pin_RF, addressRF);
  wakeSensorAndSetAddress(XSHUT_pin_RB, addressRB);
  wakeSensorAndSetAddress(XSHUT_pin_F, addressF);
  wakeSensorAndSetAddress(XSHUT_pin_B, addressB);
}

void wakeSensorAndSetAddress(int pin, uint8_t newAddress) {
  digitalWrite(pin, HIGH);
  delay(50); // Wait for the sensor to wake up
  sensor.init(true);
  delay(100); // Time for sensor to initialize
  sensor.setAddress(newAddress); // Set the new address
  sensor.setTimeout(500); // Set timeout for sensor reading
  delay(10);
}

int getSensorData(String sensorID) {
  if (sensorID == "RF") {
    sensor.setAddress(addressRF);
  } else if (sensorID == "RB") {
    sensor.setAddress(addressRB);
  } else if (sensorID == "F") {
    sensor.setAddress(addressF);
  } else if (sensorID == "B") {
    sensor.setAddress(addressB);
  } else {
    Serial.println("Invalid sensor ID");
    return -1; // Return an error code or invalid distance
  }

  int distance = sensor.readRangeSingleMillimeters();
  if (sensor.timeoutOccurred()) {
    Serial.print(sensorID + " TIMEOUT");
    return -1; // Return an error code or invalid distance
  }
  return distance;
}

void loop() {
  // Example usage
  Serial.print("RF Distance: ");
  Serial.println(getSensorData("RF"));

  Serial.print("RB Distance: ");
  Serial.println(getSensorData("RB"));

  Serial.print("F Distance: ");
  Serial.println(getSensorData("F"));

  Serial.print("B Distance: ");
  Serial.println(getSensorData("B"));

  delay(1000); // Delay between readings
}
