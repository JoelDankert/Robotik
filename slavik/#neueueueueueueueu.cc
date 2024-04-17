
#include <Adafruit_TCS34725.h>
#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>

// TOF Sensor objects
VL53L0X FF;
VL53L0X RF;
VL53L0X RB;
VL53L0X BB;

// TOF Sensor pins
const int XSHUT_pin_RF = 25;  // Example pin for Right Front sensor
const int XSHUT_pin_RB = 26;  // Example pin for Right Back sensor
const int XSHUT_pin_F = 24;   // Example pin for Front sensor
const int XSHUT_pin_B = 27;   // Example pin for Back sensor

// TOF Sensor addresses
const uint8_t addressRF = 0x30;  // Right Front
const uint8_t addressRB = 0x31;  // Right Back
const uint8_t addressF = 0x32;   // Front
const uint8_t addressB = 0x33;   // Back

// TOF Sensor offsets
const int Foffset = -1;
const int RBoffset = -1;
const int RFoffset = -1;
const int Boffset = -1;

// Motor pins
#define MOTOR1_DIR 4
#define MOTOR1_SPEED 3
#define MOTOR2_DIR 12
#define MOTOR2_SPEED 11
#define MOTOR3_DIR 8
#define MOTOR3_SPEED 5
#define MOTOR4_DIR 7
#define MOTOR4_SPEED 6

// Color detection pins
#define redPin 29    // Pin connected to the red signal from the Nano
#define blackPin 30  // Pin connected to the black signal from the Nano
#define resetPin 31  // Output pin to send reset signal to the Nano

// Servo pin
#define servopin 22

// LED pins
#define pinLED 40
#define RED_PIN 51
#define GREEN_PIN 52
#define BLUE_PIN 53

// Reset function
void (*resetFunc)(void) = 0;

// Distance thresholds
int rightWallDistanceMax = 10;
int frontWallDistanceGoal = 15;
int frontWallDistanceMin = 13;
int tryWallDistanceGoal = 20;
int rightWallDistanceGoal = 6;

// Speed and time constants
float globalSpeed = 1;
int fieldSize = 30;
float timeperangle = 6;
float speedAdj = 0.2;
int rightturncancel = 13;
const float Frightturn = 0.1;
const float Fleftturnspeed = 1;

// Color detection variables
int lastred = 0;
int reddelay = 2000;

// State variables
int state = 0;

// Error detection variables
float EDcurrentchange = 30;
float EDresetvalue = 30;
int EDlast = 0;
float EDcap = 3;
float EDchangeslow = 10;
int fatalerrorcount = 0;
int fatalerrorreset = 100;
float lastFront = 0;
float frontmax = 3;

// Debug flags
bool debug = false;
bool nocolor = false;

// Blink variables
unsigned long previousMillis = 0;  // will store last time LED was updated
const long onTime = 25;            // milliseconds of on-time
const long offTime = 100;          // milliseconds of off-time
bool ledState = false;

// Servo object
Servo dropoff;
