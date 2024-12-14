#ifndef WALL_FOLLOW_H
#define WALL_FOLLOW_H

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "pid.h"
// #include <Adafruit_VL53L1X.h> 

/*
TOOD: Tuning parameters:
Wall follow threshold - Complete
Corner Hanlding - Complete for 90 degrees
Vive - Incomplete
Hybrid Mode - Incomplete
*/

// Add new constants
const int OUT_OF_RANGE = -1;
const int MAX_SENSOR_RANGE = 2000;  // Maximum measurable disftance of the sensor in mm
const int WALL_FOLLOW_THRESHOLD = 160;          // Main parameter for wall following
const int MAX_STEERING_ANGLE_PERCENT = 4;       // Maximum steering angle as a percentage of the maximum
const int FRONT_COLLISION_THRESHOLD = 240;      // Minimum distance to obstacle in front
const float SHARP_TURN_ANGLE = 50;            // Maximum turn angle for corners


// Speed control constants (%)
const int NORMAL_SPEED = 50;    // Reduced normal speed for better control
const int CORNER_SPEED = 5;    // Speed for corners
const int WALL_CLOSE_SPEED = 50;  // Speed when close to wall

// Function declarations
void initToFSensors();
void readToFSensors(int &d_front, int &d_left, int &d_right);
void wallFollowLogic();
void sendSteeringCommand(int angle, char direction, int speed); // Changed from steer to sendSteeringCommand
float calculateSteeringAngle(float error);  // Add this line

#define PRINT_TOF_INTERVAL 1000
uint32_t last_tof_print_time = 0;

// Define pins
#define SDA_PIN 19    // Common SDA (Data line)
#define SCL_PIN 4     // Common SCL (Clock line)
#define XSHUT_FRONT 1 // Front sensor XSHUT pin
#define XSHUT_LEFT 5  // Left sensor XSHUT pin
#define XSHUT_RIGHT 18 // Right sensor XSHUT pin

// Adafruit_VL53L1X loxFront = Adafruit_VL53L1X(); // Uncomment for VL53L1X
Adafruit_VL53L0X loxFront = Adafruit_VL53L0X();
Adafruit_VL53L0X loxLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X loxRight = Adafruit_VL53L0X();

// PID controller for wall following - controls the steering angle
const float Kp_steering = 0.6;  // Proportional gain
const float Ki_steering = 0.0;  // Integral gain
const float Kd_steering = 0.9;  // Derivative gain
PIDController pidSteering(Kp_steering, Ki_steering, Kd_steering, 10, -MAX_STEERING_ANGLE_PERCENT, MAX_STEERING_ANGLE_PERCENT);

// Add sensor status flags
bool frontSensorOK = false;
bool leftSensorOK = false;
bool rightSensorOK = false;

// Simplify the initToFSensors() function to match working implementation
void initToFSensors() {
  // Configure I2C
  delay(10);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  // Power down all sensors
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(10);

  // // Initialize Front sensor (VL53L1X)
  // digitalWrite(XSHUT_FRONT, HIGH);
  // delay(50);
  // frontSensorOK = loxFront.begin(0x29);  // VL53L1X uses 0x29 by default
  // if (frontSensorOK) {
  //   loxFront.startRanging();
  //   loxFront.setTimingBudget(200); 
  // } else {
  //   Serial.println("Failed to initialize front sensor");
  // }

  // Initialize Front sensor
  digitalWrite(XSHUT_FRONT, HIGH);
  delay(50);
  frontSensorOK = loxFront.begin(0x30);
  if (!frontSensorOK) {
    Serial.println("Failed to initialize front sensor");
  }


  // Initialize Left sensor
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(50);
  leftSensorOK = loxLeft.begin(0x31);
  if (!leftSensorOK) {
    Serial.println("Failed to initialize left sensor");
  }

  // Initialize Right sensor
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(50);
  rightSensorOK = loxRight.begin(0x32);
  if (!rightSensorOK) {
    Serial.println("Failed to initialize right sensor");
  }

  // Check if at least one sensor is working
  if (!frontSensorOK && !leftSensorOK && !rightSensorOK) {
    Serial.println("No sensors initialized! System halted.");
    while(1);
  }
  
  Serial.println("System initialized with working sensors!");
}

int getFrontDistance() {
  if (!frontSensorOK) return OUT_OF_RANGE;
  VL53L0X_RangingMeasurementData_t measure;
  loxFront.rangingTest(&measure, false);
  return (measure.RangeStatus != 4) ? measure.RangeMilliMeter : OUT_OF_RANGE;
}

int getLeftDistance() {
  if (!leftSensorOK) return OUT_OF_RANGE;
  VL53L0X_RangingMeasurementData_t measure;
  loxLeft.rangingTest(&measure, false);
  return (measure.RangeStatus != 4) ? measure.RangeMilliMeter : OUT_OF_RANGE;
}

int getRightDistance() {
  if (!rightSensorOK) return OUT_OF_RANGE;
  VL53L0X_RangingMeasurementData_t measure;
  loxRight.rangingTest(&measure, false);
  return (measure.RangeStatus != 4) ? measure.RangeMilliMeter : OUT_OF_RANGE;
}
// Function to read distances from all sensors
void readToFSensors(int &d_front, int &d_left, int &d_right) {
  d_front = getFrontDistance();
  d_left = getLeftDistance();
  d_right = getRightDistance();
}

// Function to read effective distance from a sensor
int getEffectiveDistance(int distance) {
  return (distance == OUT_OF_RANGE || distance > 8000) ? MAX_SENSOR_RANGE : distance;
}

// Function to detect and avoid any obstacles in front
bool frontObstacleAvoidance(bool debugPrint = false) {
  int d_front, d_left, d_right;
  readToFSensors(d_front, d_left, d_right);

  int effective_front = getEffectiveDistance(d_front);
  int effective_left = getEffectiveDistance(d_left);
  int effective_right = getEffectiveDistance(d_right);

  if (debugPrint) {
    Serial.print("Front: "); Serial.println(d_front == OUT_OF_RANGE ? "OOR" : String(effective_front));
    Serial.print("Left: "); Serial.println(d_left == OUT_OF_RANGE ? "OOR" : String(effective_left));
    Serial.print("Right: "); Serial.println(d_right == OUT_OF_RANGE ? "OOR" : String(effective_right));
  }

  bool detectedObstacle = false;
  if (effective_front <= FRONT_COLLISION_THRESHOLD) {
    char direction = 'F';
    int speed = NORMAL_SPEED;  // Default speed
    float steering_angle = 0;

    // Obstacle detected
    detectedObstacle = true;

    // Corner handling
      if (effective_left > effective_right) {
          direction = 'R';
          steering_angle = SHARP_TURN_ANGLE;
          if (debugPrint) {
              Serial.print("Turning right - steering angle: ");
              Serial.println(steering_angle);
          }
      } else {
          direction = 'L';
          steering_angle = SHARP_TURN_ANGLE;
          if (debugPrint) {
              Serial.print("Turning left - steering angle: ");
              Serial.println(steering_angle);
          }
      }
      speed = CORNER_SPEED;

      // Send steering command
      sendSteeringCommand((int)steering_angle, direction, speed);
  }
  
  return detectedObstacle;
}

// Function to detect if wall on the specified side, if follow == true, then apply steering
// angle to follow the wall. If follow == false, then just detect if the wall is
// present below the WALL_FOLLOW_THRESHOLD
bool wallDetection(bool leftWall = true, bool follow = false, bool printDebug = false) {
  int dist;
  if (leftWall)  dist = getLeftDistance();
  else           dist = getRightDistance();

  int effective_distance = getEffectiveDistance(dist);

  if (follow || (effective_distance <= WALL_FOLLOW_THRESHOLD)) {
    char direction = 'F';
    int speed = NORMAL_SPEED;  // Default speed

    // Compute steering angle based on effective distance using PID controller
    float steering_angle = pidSteering.compute(WALL_FOLLOW_THRESHOLD, effective_distance);

    if (steering_angle > 0) {
      if (leftWall)   direction = 'R';
      else            direction = 'L';
      
      speed = WALL_CLOSE_SPEED;
      if (printDebug) {
          Serial.print("Turning ");             Serial.print(leftWall ? "right" : "left");
          Serial.print(" - steering angle: ");  Serial.println(steering_angle);
      }
    } else if (steering_angle < 0) {
      if (leftWall)   direction = 'L';
      else            direction = 'R';
        
      speed = WALL_CLOSE_SPEED;
      steering_angle = -steering_angle;  // Make angle positive
      if (printDebug) {
          Serial.print("Turning left - steering angle: ");
          Serial.println(steering_angle);
      }
    } else {
        direction = 'F';
        speed = NORMAL_SPEED;
        if (printDebug) {
            Serial.println("Moving forward");
        }
        steering_angle = 0;
    }

    // Send steering command
    sendSteeringCommand((int)steering_angle, direction, speed);
    return true;
  }

  return false;
}

// Simplify the wallFollowLogic function
void wallFollowLogic(
  bool leftWall = true // Wall to follow
) {
    bool printDebug = false;
    if (millis() - last_tof_print_time > PRINT_TOF_INTERVAL) {
      printDebug = true;
      last_tof_print_time = millis();
    }

    // Default values for movement
    float steering_angle = 0;
    char direction = 'F';
    int speed = NORMAL_SPEED;  // Default speed

    // Reimplement the wall following logic using helper functions
    if (frontObstacleAvoidance(printDebug))               return;
    else if (wallDetection(leftWall, true, printDebug))   return;
    else
      // No obstacles detected
      sendSteeringCommand((int)steering_angle, direction, speed);
}

#endif
