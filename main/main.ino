/*
Code for integrating the sensor and motor control code in a single ESP. 
The code is a combination of the sensor code and the auto code, with the web server code from the sensor.ino
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "web.h"          
#include "pid.h"
// #include "rgb.h"
#include "top_hat.h"
#include "planning.h"

// ---------------- CONSTANTS & DEFINES ----------------
// From auto code
#define LEDC_RESOLUTION_BITS 10
#define LEDC_RESOLUTION ((1 << LEDC_RESOLUTION_BITS) - 1)
#define LEDC_FREQUENCY 50  // Frequency in Hz
#define EXTRA_CARE 1 // whether to make motor stop before switching direction
#define MOTOR_STOP_DELAY 10

#define RPM_TO_RAD_PER_SEC 0.10471975512
#define MAX_RPM 110
#define MAX_WHEEL_VELOCTY (MAX_RPM * RPM_TO_RAD_PER_SEC)
#define WHEEL_RADIUS 36
#define WHEEL_BASE 185
#define MAX_ANGULAR_VELOCITY ((MAX_WHEEL_VELOCTY * WHEEL_RADIUS)/WHEEL_BASE)
#define PRINT_AUTO_FREQUENCY 1000

// For servo
const int servoPWMPin = 3;
const int servoMinAngle = 0;
const int servoMaxAngle = 180;
const int servoDefaultAngle = 90;
const int servoSwingAngle = 45;
const int servoMinPulse = 500;  // microseconds
const int servoMaxPulse = 2500; // microseconds
const int servoMinDuty = (servoMinPulse * (1 << LEDC_RESOLUTION_BITS)) / 20000;
const int servoMaxDuty = (servoMaxPulse * (1 << LEDC_RESOLUTION_BITS)) / 20000;

// Motor pins
const int pwmPinLeft = 21;  
const int pwmPinRight = 33; 
const int dirPinLeft = 4;  
const int dirPinRight = 5; 

// Encoders
const int encoderPinLeftA = 21;
const int encoderPinLeftB = 33;
const int encoderPinRightA = 11;
const int encoderPinRightB = 12;
const int pulsesPerRevolution = 48;
const int gearRatio = 100;

// Top Hat
// TOP_HAT_READ_INTERVAL and related functions are in top_hat.h and sensor_read.h

// ---------------- GLOBAL VARIABLES ----------------
int desiredLeftPWM = 0;
int desiredLeftDirection = 1;
int desiredRightPWM = 0;
int desiredRightDirection = 1;

int controlSignalLeft = 0;
int controlSignalRight = 0;
float KP = 1.0;
float KI = 0.1;
float KD = 0.0;
int ENABLE_CONTROL = 1;
int defaultPWM = 500;

volatile int leftPulseCount = 0;
volatile int rightPulseCount = 0;
volatile int leftDirectionEnc = 1;   
volatile int rightDirectionEnc = 1;  
int lastEncodedLeft = 0;
int lastEncodedRight = 0;

volatile unsigned long prevRpmCalcTime = 0;
const unsigned long rpmCalcInterval = 50;
unsigned int leftRPM = 0;
unsigned int rightRPM = 0;

PIDController leftPID(KP, KI, KD, 4000, -LEDC_RESOLUTION, LEDC_RESOLUTION);
PIDController rightPID(KP, KI, KD, 4000, -LEDC_RESOLUTION, LEDC_RESOLUTION);

bool autonomousMode = true;  
bool servoOff = false;
bool swingServo = true;
int swingSpeed = 1; // servo swing speed
uint32_t last_auto_print_time = 0;
bool printDebug = false;

// Variables from sensor code
Planner planner;
uint32_t wifi_packets = 0;

// WiFi configuration (from sensor code)
const char* ssid = "GM Lab Public WIFI";
const char* password = "";
IPAddress myIP(192, 168, 1, 101);
WebServer server(81);

volatile unsigned long serverPrevTime = 0;
const unsigned long serverInterval = 50;

// Variables for steering commands (simulate what used to come from UDP/I2C)
int receivedAngle = 0;
char receivedDirection = 'F';
int receivedSpeed = 50;
int receivedServo = 1; // 1 for on, 0 for off

// ---------------- FUNCTION DECLARATIONS ----------------
void IRAM_ATTR updateLeftEncoder();
void IRAM_ATTR updateRightEncoder();
void readEncoderValue(int encoder_A_val, int encoder_B_val, int& last_encoded_val, int& direction, int& pulse_count);

void stopCar();
void moveForward();
void turnLeft();
void turnRight();

void prepareIdealMotorSignals(float linear_velocity, float angular_velocity, int& left_pwm,int& left_dir,int& right_pwm,int& right_dir);
void prepareControlledMotorSignals(int ideal_left_pwm, int ideal_right_pwm, int& controlled_left_pwm, int& controlled_right_pwm);
void convertAngularVelocityToPWM(float omega, int& pwm_ref, int& direction);
float mapf(float value, float inMin, float inMax, float outMin, float outMax);
void sendMotorSignals(int left_pwm, int left_direction, int right_pwm, int right_direction);

void rpmCalculation();
void updateControlSignals();
void sendSteeringCommand(int angle, char direction, int speed, int servo);

void steer(int angle, char direction, int speed);
void handleRoot();
void handleSetMode();
void handleSetMotor();
void handleSetServo();
void setServo(int servoSignal);
unsigned int angleToDuty(int angle);
void setServoAngle(int angle);
void swingServoFunction(int swingSpeed);
void handleServo(bool servoOff, bool swingServo, int swingSpeed);
void handleSetAutonomous();


// Functions from planning and sensor
// readToFSensors, top hat reading are in included .h files.
// Planner logic calls planner.planLogic()
// We must read top hat data regularly
// We must call sendTopHatData(wifi_packets) if implemented similarly as in original code.

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  pinMode(dirPinLeft, OUTPUT);
  pinMode(dirPinRight, OUTPUT);
  pinMode(pwmPinLeft, OUTPUT);
  pinMode(pwmPinRight, OUTPUT);
  pinMode(encoderPinLeftA, INPUT);
  pinMode(encoderPinLeftB, INPUT);
  pinMode(encoderPinRightA, INPUT);
  pinMode(encoderPinRightB, INPUT);
  
  digitalWrite(dirPinLeft, LOW);
  digitalWrite(dirPinRight, LOW);

  ledcAttach(pwmPinLeft, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);
  ledcAttach(pwmPinRight, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);
  ledcWrite(pwmPinLeft, 0);
  ledcWrite(pwmPinRight, 0);
  delay(1000);
  ledcWrite(pwmPinLeft, 500);
  ledcWrite(pwmPinRight, 500);
}

void loop() {

}