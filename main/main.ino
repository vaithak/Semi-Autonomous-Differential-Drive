/*
Code for integrating the sensor and motor control code in a single ESP. 
The code is a combination of the sensor code and the auto code, with the web server code from the sensor.ino
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <Adafruit_VL53L0X.h>
#include "web.h"          
#include "pid.h"
#include "rgb.h"
#include "top_hat.h"
#include "planning.h"
#include "math.h"

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
const int servoPWMPin = 10;
const int servoMinAngle = 0;
const int servoMaxAngle = 180;
const int servoDefaultAngle = 90;
const int servoSwingAngle = 45;
const int servoMinPulse = 500;  // microseconds
const int servoMaxPulse = 2500; // microseconds
const int servoMinDuty = (servoMinPulse * (1 << LEDC_RESOLUTION_BITS)) / 20000;
const int servoMaxDuty = (servoMaxPulse * (1 << LEDC_RESOLUTION_BITS)) / 20000;

// Motor pins
const int pwmPinLeft = 2;  
const int pwmPinRight = 1; 
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
IPAddress local_IP(192, 168, 1, 105);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
WebServer server(80);

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
  setupRGB();

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

  attachInterrupt(digitalPinToInterrupt(encoderPinLeftA), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinLeftB), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinRightA), updateRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinRightB), updateRightEncoder, CHANGE);

  // Setup Wi-Fi in AP mode
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ssid, password, 4);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  // Init planner
  planner.setup();

  // Set server routes
  server.on("/", handleRoot);
  server.on("/setMode", handleSetMode);
  server.on("/setMotor", handleSetMotor);
  server.on("/setServo", handleSetServo);
  server.on("/setAutonomous", handleSetAutonomous);
  server.begin();
  Serial.println("HTTP server started");

  initTopHat();
  
  // Initialize servo
  ledcAttach(servoPWMPin, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);
  setServoAngle(servoDefaultAngle);
  swingServo = true;
  servoOff = false;
}

// ---------------- LOOP ----------------
void loop() {
  handleRGB();
  server.handleClient();

  printDebug = false;
  if (millis() - last_auto_print_time > PRINT_AUTO_FREQUENCY) {
    printDebug = true;
    last_auto_print_time = millis();
  }

  // Read top hat data periodically
  static unsigned long topHatLastRead = 0;
  if (millis() - topHatLastRead > TOP_HAT_READ_INTERVAL) {
    uint8_t topHatData = readTopHatData();
    if (printDebug) {
      Serial.printf("Top hat data: %d\n", topHatData);
    }
    // If we had logic to send top hat data via wifi_packets, we can do it now
    // For now, just reset wifi_packets
    wifi_packets = 0;
    topHatLastRead = millis();
  }

  // Run planner logic if autonomous mode
  if (autonomousMode) {
    planner.planLogic();
    // planner may set some internal steering commands
    // The planner presumably calls steer(...) or sets global variables
  }

  // RPM calculation and PID update at intervals
  unsigned long currentTime = millis();
  if (currentTime - prevRpmCalcTime > rpmCalcInterval) {
    rpmCalculation();
    leftPulseCount = 0;
    rightPulseCount = 0;
    updateControlSignals();
    prevRpmCalcTime = currentTime;
  }

  // Here we simulate receiving steering commands. In fully integrated mode,
  // either planner sets them or web handlers set them.
  // We'll just call steer() and setServo() with the current globals.
  steer(receivedAngle, receivedDirection, receivedSpeed);
  setServo(receivedServo);

  // Prepare and send motor signals
  int controlled_left_pwm, controlled_right_pwm;
  prepareControlledMotorSignals(desiredLeftPWM, desiredRightPWM, controlled_left_pwm, controlled_right_pwm);
  if (printDebug) {
    Serial.printf("Control signals: Left: %d, Right: %d\n", controlSignalLeft, controlSignalRight);
    Serial.printf("Controlled Left PWM: %d, Controlled Right PWM: %d\n", controlled_left_pwm, controlled_right_pwm);
  }
  sendMotorSignals(controlled_left_pwm, desiredLeftDirection, controlled_right_pwm, desiredRightDirection);

  handleServo(servoOff, swingServo, swingSpeed);
}

// ---------------- ENCODER INTERRUPTS ----------------
void IRAM_ATTR updateLeftEncoder() {
  int left_encoder_A = digitalRead(encoderPinLeftA);
  int left_encoder_B = digitalRead(encoderPinLeftB);
  int current_direction = 0;
  int current_pulse_count = 0;
  readEncoderValue(left_encoder_A, left_encoder_B, lastEncodedLeft, current_direction, current_pulse_count);
  leftPulseCount += current_pulse_count;
  // Adjust direction if needed
  leftDirectionEnc = current_direction * -1;
}

void IRAM_ATTR updateRightEncoder() {
  int right_encoder_A = digitalRead(encoderPinRightA);
  int right_encoder_B = digitalRead(encoderPinRightB);
  int current_direction = 0;
  int current_pulse_count = 0;
  readEncoderValue(right_encoder_A, right_encoder_B, lastEncodedRight, current_direction, current_pulse_count);
  rightPulseCount += current_pulse_count;
  rightDirectionEnc = current_direction;
}

// ---------------- HELPER FUNCTIONS ----------------
void stopCar() {
  ledcWrite(pwmPinLeft, 0);
  ledcWrite(pwmPinRight, 0);
}

void moveForward() {
  sendMotorSignals(defaultPWM, LOW, defaultPWM, LOW);
}

void turnLeft() {
  sendMotorSignals(defaultPWM / 2, LOW, defaultPWM, LOW);
}

void turnRight() {
  sendMotorSignals(defaultPWM, LOW, defaultPWM / 2, LOW);
}

void prepareIdealMotorSignals(float linear_velocity, float angular_velocity, int& left_pwm,int& left_dir,int& right_pwm,int& right_dir) {
  float omega_l = (linear_velocity - angular_velocity * WHEEL_BASE / 2) / WHEEL_RADIUS;
  float omega_r = (linear_velocity + angular_velocity * WHEEL_BASE / 2) / WHEEL_RADIUS;
  convertAngularVelocityToPWM(omega_l, left_pwm, left_dir);
  convertAngularVelocityToPWM(omega_r, right_pwm, right_dir);
  left_pwm = min(max(left_pwm, 0), LEDC_RESOLUTION);
  right_pwm = min(max(right_pwm, 0), LEDC_RESOLUTION);
}

void prepareControlledMotorSignals(int ideal_left_pwm,int ideal_right_pwm,int& controlled_left_pwm,int& controlled_right_pwm) {
  controlled_left_pwm = ideal_left_pwm;
  controlled_right_pwm = ideal_right_pwm;
  if (ENABLE_CONTROL) {
    controlled_left_pwm += controlSignalLeft;
    controlled_right_pwm += controlSignalRight;
  }
  controlled_left_pwm = min(max(controlled_left_pwm, 0), LEDC_RESOLUTION);
  controlled_right_pwm = min(max(controlled_right_pwm, 0), LEDC_RESOLUTION);
}

void convertAngularVelocityToPWM(float omega, int& pwm_ref, int& direction) {
  if (omega > 0.0) {
    direction = LOW;
    pwm_ref = (int)mapf(omega, 0, MAX_WHEEL_VELOCTY, 0, (float)LEDC_RESOLUTION);
  } else if (omega < 0.0) {
    direction = HIGH;
    pwm_ref = (int)mapf(-omega, 0, MAX_WHEEL_VELOCTY, 0, (float)LEDC_RESOLUTION);
  } else {
    pwm_ref = 0;
  }
}

float mapf(float value, float inMin, float inMax, float outMin, float outMax) {
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void sendMotorSignals(int left_pwm, int left_direction, int right_pwm, int right_direction) {
#if EXTRA_CARE
  if (left_direction != desiredLeftDirection) {
    ledcWrite(pwmPinLeft, 0);
  }
  if (right_direction != desiredRightDirection) {
    ledcWrite(pwmPinRight, 0);
  }
  delay(MOTOR_STOP_DELAY);
#endif
  digitalWrite(dirPinLeft, left_direction);
  digitalWrite(dirPinRight, right_direction);
  if (printDebug) {
    Serial.printf("Actual Left PWM: %d, Actual Right PWM: %d\n", left_pwm, right_pwm);
  }
  ledcWrite(pwmPinLeft, left_pwm);
  ledcWrite(pwmPinRight, right_pwm);
  desiredLeftDirection = left_direction;
  desiredRightDirection = right_direction;
}

void rpmCalculation() {
  unsigned long elapsed = millis() - prevRpmCalcTime;
  leftRPM = (abs(leftPulseCount) * 60000UL) / ((pulsesPerRevolution) * (elapsed));
  rightRPM = (abs(rightPulseCount) * 60000UL) / ((pulsesPerRevolution) * (elapsed));

  leftRPM = (leftRPM * 2) / gearRatio;
  rightRPM = (rightRPM * 2) / gearRatio;
}

void updateControlSignals() {
  int current_pwm_left = map(leftRPM, 0, MAX_RPM, 0, LEDC_RESOLUTION);
  int current_pwm_right = map(rightRPM, 0, MAX_RPM, 0, LEDC_RESOLUTION);

  if (printDebug) {
    Serial.printf("Current RPM - Left: %d (dir: %d), Right: %d (dir: %d)\n", leftRPM, leftDirectionEnc, rightRPM, rightDirectionEnc);
    Serial.printf("Current PWM - Left: %d, Right: %d\n", current_pwm_left, current_pwm_right);
  }

  controlSignalLeft = leftPID.compute(desiredLeftPWM, current_pwm_left);
  controlSignalRight = rightPID.compute(desiredRightPWM, current_pwm_right);
}

void readEncoderValue(int encoder_A_val,int encoder_B_val,int& last_encoded_val,int& direction,int& pulse_count) {
  int MSB = encoder_A_val;
  int LSB = encoder_B_val;
  int encoded = (MSB << 1) | LSB;
  int sum = (last_encoded_val << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    direction = 1;
    pulse_count++;
  } else if (sum == 0b1110 || sum == 0b1000 || sum == 0b0001 || sum == 0b0111) {
    direction = -1;
    pulse_count--;
  }

  last_encoded_val = encoded;
}

void sendSteeringCommand(int angle, char direction, int speed, int servo) {
  // Here you integrate with your existing steering code.
  // For example, if you have a steer() function and setServo() function:
  steer(angle, direction, speed);
  setServo(servo);
}


// Steering logic
void steer(int angle, char direction, int speed) {
  float maxSteeringAngle = 50;
  float moveSpeed = (speed / 100.0) * (MAX_WHEEL_VELOCTY * WHEEL_RADIUS); // Convert speed% to linear velocity
  float angular_velocity = (angle / maxSteeringAngle) * MAX_ANGULAR_VELOCITY;

  if (direction == 'L') {
    // positive angular velocity
  } else if (direction == 'R') {
    angular_velocity = -angular_velocity;
  } else {
    angular_velocity = 0;
  }

  int left_pwm, left_dir, right_pwm, right_dir;
  prepareIdealMotorSignals(moveSpeed, angular_velocity, left_pwm, left_dir, right_pwm, right_dir);
  if (printDebug) {
    Serial.printf("Steering: angle=%d, direction=%c, speed=%d\n", angle, direction, speed);
    Serial.printf("Desired Left: PWM=%d, Direction=%d\n", left_pwm, left_dir);
    Serial.printf("Desired Right: PWM=%d, Direction=%d\n", right_pwm, right_dir);
  }
  desiredLeftPWM = left_pwm;
  desiredLeftDirection = left_dir;
  desiredRightPWM = right_pwm;
  desiredRightDirection = right_dir;
}

// Web Handlers
void handleRoot() {
  server.send_P(200, "text/html", WEBPAGE);
}

void handleSetMode() {
  if (server.hasArg("mode")) {
    String mode = server.arg("mode");
    // Just handle operational modes here, e.g.:
    if (mode == "leftWallFollow") {
      planner.setWaypointsAndMode(0,0,"leftWallFollow");
    } else if (mode == "rightWallFollow") {
      planner.setWaypointsAndMode(0,0,"rightWallFollow");
    } else if (mode == "attackRampBlueTower") {
      planner.setWaypointsAndMode(0,0,"attackRampBlueTower");
    } else if (mode == "attackRampRedTower") {
      planner.setWaypointsAndMode(0,0,"attackRampRedTower");
    } else if (mode == "attackGroundNexusRight") {
      planner.setWaypointsAndMode(0,0,"attackGroundNexusRight");
    } else if (mode == "attackGroundNexusLeft") {
      planner.setWaypointsAndMode(0,0,"attackGroundNexusLeft");
    } else if (mode == "attackBlueGroundNexusCenter") {
      planner.setWaypointsAndMode(0,0,"attackBlueGroundNexusCenter");
    } else if (mode == "attackRedGroundNexusCenter") {
      planner.setWaypointsAndMode(0,0,"attackRedGroundNexusCenter");
    } else if (mode == "gridMode" && server.hasArg("x") && server.hasArg("y")) {
      int x = server.arg("x").toInt();
      int y = server.arg("y").toInt();
      planner.setWaypointsAndMode(x, y, "gridMode");
    } else {
      server.send(400, "text/plain", "Unknown mode");
      return;
    }
    server.send(200, "text/plain", "Mode updated");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}


void handleSetMotor() {
  // Manual control endpoint
  if (server.hasArg("speed") && server.hasArg("forwardBackward") && server.hasArg("turnRate")) {
    float motor_speed = server.arg("speed").toFloat(); // 0-100
    String forward_backward = server.arg("forwardBackward");
    float turn_rate = server.arg("turnRate").toFloat(); // -50 to 50
    if (forward_backward == "Forward") {
      // direction forward
      // turn_rate positive = left turn, negative = right turn
      int angle = (int)fabs(turn_rate);
      char dir = (turn_rate < 0) ? 'R' : (turn_rate > 0) ? 'L' : 'F';
      steer(angle, dir, (int)motor_speed);
    } else {
      // backward
      // treat backward as negative speed
      motor_speed = -motor_speed;
      int angle = (int)fabs(turn_rate);
      char dir = (turn_rate < 0) ? 'R' : (turn_rate > 0) ? 'L' : 'F';
      steer(angle, dir, (int)motor_speed);
    }
    server.send(200, "text/plain", "Motor command updated");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

void handleSetServo() {
  if (server.hasArg("servo")) {
    String servoState = server.arg("servo");
    if (servoState == "off") {
      servoOff = true;
      swingServo = false;
    } else if (servoState == "on") {
      servoOff = false;
      swingServo = true;
    }
    server.send(200, "text/plain", "Servo parameters updated");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

void setServo(int servoSignal) {
  if (servoSignal == 0) {
    servoOff = true;
    swingServo = false;
  } else {
    servoOff = false;
    swingServo = true;
  }
  handleServo(servoOff, swingServo, swingSpeed);
}

unsigned int angleToDuty(int angle) {
  if (angle < servoMinAngle) angle = servoMinAngle;
  if (angle > servoMaxAngle) angle = servoMaxAngle;
  return map(angle, servoMinAngle, servoMaxAngle, servoMinDuty, servoMaxDuty);
}

void setServoAngle(int angle) {
  unsigned int duty = angleToDuty(angle);
  ledcWrite(servoPWMPin, duty);
}

void swingServoFunction(int swingSpeed) {
  static int servoAngle = servoDefaultAngle - servoSwingAngle;
  static int servoIncrement = swingSpeed; 
  setServoAngle(servoAngle);
  servoAngle += servoIncrement;
  if (servoAngle >= servoDefaultAngle + servoSwingAngle || servoAngle <= servoDefaultAngle - servoSwingAngle) {
    servoIncrement = -servoIncrement;
  }
}

void handleServo(bool servoOff, bool swingServo, int swingSpeed) {
  if (servoOff) {
    setServoAngle(servoMinAngle); 
  } else if (swingServo) {
    swingServoFunction(swingSpeed);
  } else {
    setServoAngle(servoDefaultAngle);
  }
}

void handleSetAutonomous() {
  if (server.hasArg("autonomous")) {
    String val = server.arg("autonomous");
    if (val == "true") {
      autonomousMode = true;
      server.send(200, "text/plain", "Autonomous mode enabled");
    } else {
      autonomousMode = false;
      server.send(200, "text/plain", "Manual mode enabled");
    }
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}