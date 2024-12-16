/*
MEAM 5100 Final Project - Auto ESP Code
Author: Team 27
The auto ESP code is responsible for controlling the car based on sensor data and user input.
Modified based on controller.ino from Lab 4.
*/

#include <WiFi.h>
#include <WiFiUdp.h>
// #include <web.h>
#include <pid.h>
// #include <WebServer.h>  
#include "rgb.h"

#define LEDC_RESOLUTION_BITS 10
#define LEDC_RESOLUTION ((1 << LEDC_RESOLUTION_BITS) - 1)
#define LEDC_FREQUENCY 50  // Frequency in Hz
#define EXTRA_CARE 1 // whether to make motor stop before switching direction
#define MOTOR_STOP_DELAY 10

#define RPM_TO_RAD_PER_SEC 0.10471975512                   // Conversion factor from RPM to rad/s
#define MAX_RPM 110                                        // Maximum RPM of the motor - on average load.
#define MAX_WHEEL_VELOCTY (MAX_RPM * RPM_TO_RAD_PER_SEC)   // Maximum wheel velocity in rad/s
#define WHEEL_RADIUS 36                                    // Wheel radius in millimeters

// V = (wl*r + wr*r) / 2
#define MAX_LINEAR_VELOCTY (MAX_WHEEL_VELOCTY * WHEEL_RADIUS) // Maximum linear velocity in m/s

// w_combined = (wl - wr)*r/L
#define WHEEL_BASE 185  // Distance between wheels in millimeters
#define MAX_ANGULAR_VELOCITY ((MAX_WHEEL_VELOCTY * WHEEL_RADIUS)/WHEEL_BASE)

#define SAFE_DISTANCE 300 // Safe distance in millimeters

#define PRINT_AUTO_FREQUENCY 1000
uint32_t last_auto_print_time = 0;

// Wi-Fi network name and password
const char* ssid = "GM Lab Public WIFI";  // Wi-Fi network name
const char* password = "";   // Wi-Fi password (empty for no password)

// IP configuration for AP mode
IPAddress local_IP(192, 168, 1, 104);  // ESP32's IP address
IPAddress gateway(192, 168, 1, 104);   // Gateway IP (same as ESP32 in AP mode)
IPAddress subnet(255, 255, 255, 0);    // Subnet mask

WiFiUDP udp;
const unsigned int localUdpPort = 8888; // Port to listen on
char incomingPacket[5];  // Buffer for incoming packets
// volatile unsigned long serverPrevTime = 0;
// const unsigned long serverInterval = 50;

// Define the pins for the motor control
// Inverter 4 -> IN 2, Inverter 2 -> IN 4
const int pwmPinLeft = 4;  // PWM output pin (connected to EN pin on H-Bridge)
const int pwmPinRight = 5;  // PWM output pin (connected to EN pin on H-Bridge)
const int dirPinLeft = 18;  // Direction control pin 1 (for left motor) -> Inverter 1, IN 3
const int dirPinRight = 19;  // Direction control pin 2 (for right motor) -> Inverter 3, IN 1

// Gloal variable to store current desired PWM for left and right motors
int desiredLeftPWM = 0;
int desiredLeftDirection = 1;
int desiredRightPWM = 0;
int desiredRightDirection = 1;

// Global variables to store the control signals for the left and right motors
int controlSignalLeft = 0;
int controlSignalRight = 0;
float KP = 1.0;
float KI = 0.1;
float KD = 0.0;
int ENABLE_CONTROL = 1;

// Define the pins for the encoder A and B channels
const int encoderPinLeftA = 7;
const int encoderPinLeftB = 6;
const int encoderPinRightA = 1;
const int encoderPinRightB = 0;
const int pulsesPerRevolution = 48;
const int gearRatio = 100;

// Variables for the encoder
volatile int leftDirection = 1;   // Direction of the left motor
volatile int rightDirection = 1;  // Direction of the right motor
volatile int leftPulseCount = 0;  // Pulse count for the left encoder
volatile int rightPulseCount = 0; // Pulse count for the right encoder
int lastEncodedLeft = 0;          // Store the last encoded state (A and B)
int lastEncodedRight = 0;         // Store the last encoded state (A and B)

// Rpm calculation variables
volatile unsigned long prevRpmCalcTime = 0;
const unsigned long rpmCalcInterval = 50;
unsigned int leftRPM = 0;
unsigned int rightRPM = 0;

// PIDs for both motors
PIDController leftPID(KP, KI, KD, 4000, -LEDC_RESOLUTION, LEDC_RESOLUTION);
PIDController rightPID(KP, KI, KD, 4000, -LEDC_RESOLUTION, LEDC_RESOLUTION);

#define DEBUG 1

// Variables to store received commands
volatile int receivedAngle = 0;
volatile int receivedSpeed = 0;
char receivedDirection[10] = "FORWARD";  // Remove volatile
volatile bool newCommandReceived = false;

// Add global variable for default PWM
int defaultPWM = 500;  // Default PWM value that can be changed via webpage

// Add WebServer instance
// WebServer server(80);  // Add this with other global variables at the top

// Add to global variables
bool autonomousMode = true;  // Default to autonomous mode


// Add constants for servo control
const int servoPWMPin = 10;
const int servoMinAngle = 0;
const int servoMaxAngle = 180;
const int servoDefaultAngle = 90;
const int servoSwingAngle = 45;
const int servoMinPulse = 500;  // Min pulse width in microseconds (0.5ms)
const int servoMaxPulse = 2500; // Max pulse width in microseconds (2.5ms)
const int servoMinDuty = (servoMinPulse * (1 << LEDC_RESOLUTION_BITS)) / 20000;
const int servoMaxDuty = (servoMaxPulse * (1 << LEDC_RESOLUTION_BITS)) / 20000;

// Add global variables for servo control
bool swingServo = true;
bool servoOff = false;
int healthPoints = 0;

// Add global variable for swing speed
int swingSpeed = 1; // Adjust as needed

// Interrupt function to update the encoder position, add IRAM_ATTR to run in IRAM
void IRAM_ATTR updateLeftEncoder() {
  // Read the current state of both encoder pins for left motor
  int left_encoder_A = digitalRead(encoderPinLeftA);  // Most Significant Bit
  int left_encoder_B = digitalRead(encoderPinLeftB);  // Least Significant Bit

  // Call encoder reading function for left motor
  int current_direction = 0;
  int current_pulse_count = 0;
  readEncoderValue(left_encoder_A, left_encoder_B, lastEncodedLeft, current_direction, current_pulse_count);
  leftPulseCount += current_pulse_count;
  leftDirection = current_direction * -1; // Motor is the opposite direction for wheels to go in the same direction
}

// Interrupt function to update the encoder position, add IRAM_ATTR to run in IRAM
void IRAM_ATTR updateRightEncoder() {
  // Read the current state of both encoder pins for right motor
  int right_encoder_A = digitalRead(encoderPinRightA);  // Most Significant Bit
  int right_encoder_B = digitalRead(encoderPinRightB);  // Least Significant Bit

  // Call encoder reading function for right motor
  int current_direction = 0;
  int current_pulse_count = 0;
  readEncoderValue(right_encoder_A, right_encoder_B, lastEncodedRight, current_direction, current_pulse_count);
  rightPulseCount += current_pulse_count;
  rightDirection = current_direction;
}

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

  // Attach the interrupt service routine for the encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderPinLeftA), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinLeftB), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinRightA), updateRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinRightB), updateRightEncoder, CHANGE);

  // Set up Access Point
  Serial.println("Setting up Access Point...");
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ssid, password, 11);
  // Wi-Fi setup as STA mode
  // WiFi.mode(WIFI_MODE_STA);
  // WiFi.config(local_IP, gateway, subnet);
  // WiFi.begin(ssid, password, 2);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  udp.begin(localUdpPort);
  Serial.print("UDP server started at internal IP: ");
  Serial.print(local_IP);
  Serial.print("\n");

  // Update setup to include new endpoints
  // server.on("/", handleRoot);
  // server.on("/setMode", handleSetMode);
  // server.on("/setMotor", handleSetMotor);
  // server.begin();
  // Serial.println("HTTP server started");

  // Initialize UDP properly
  if(udp.begin(8888)) {
    Serial.println("UDP server started at port 8888");
  } else {
    Serial.println("UDP server failed to start");
  }

  // Initialize PWM for servo
  ledcAttach(servoPWMPin, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);

  // Set servo to default angle
  setServoAngle(servoDefaultAngle);

  // Initialize servo states
  swingServo = true;
  servoOff = false;
}

/**
 * Main loop handles:
 * 1. Web server requests
 * 2. UDP packets from sensor.ino
 * 3. Motor control based on mode (autonomous/manual)
 * 4. RPM calculations and PID control
 */
bool printDebug = true;
void loop() {
  // TODO: Send UDP Packet to sensor.ino, so that it can send packet to top hat to take off health
  // This will ONLY happen when we make a manual override 
  handleRGB();
  // Handle web server requests at regular intervals
  // if (millis() - serverPrevTime > serverInterval) {
  //   server.handleClient();
  //   serverPrevTime = millis();
  // }

  printDebug = false;
  if (millis() - last_auto_print_time > PRINT_AUTO_FREQUENCY) {
    printDebug = true;
    last_auto_print_time = millis();
  }

  // Handle UDP packets
  int packetSize = udp.parsePacket();

  if (packetSize) {
    char incomingPacket[5];
    int len = udp.read(incomingPacket, 5);
    if (len > 0) {
      incomingPacket[len] = 0;  // Null terminate
      if (printDebug) {
        Serial.printf("[Successful] Received packet: %s\n", incomingPacket);
      }
      
      // Parse Packet - Add error checking
      bool parseError = false;
      
      if (!parseError) {
        receivedSpeed = incomingPacket[0];
        receivedSpeed = receivedSpeed - 100;  // Map speed from 0 to 200 to -100 to 100
        receivedAngle = incomingPacket[1];
        char dirChar = incomingPacket[2];
        healthPoints = incomingPacket[3];
        servoOff = incomingPacket[4] == 0 ? true : false;
        Serial.print("Angle: ");
        Serial.println(receivedAngle);
        Serial.print("Speed: ");
        Serial.println(receivedSpeed);
        Serial.print("Direction: ");
        Serial.println(dirChar);

        // Convert single character to direction string
        switch((char)dirChar) {
          case 'L':
            strncpy(receivedDirection, "LEFT", sizeof(receivedDirection) - 1);
            break;
          case 'R':
            strncpy(receivedDirection, "RIGHT", sizeof(receivedDirection) - 1);
            break;
          case 'F':
            strncpy(receivedDirection, "FORWARD", sizeof(receivedDirection) - 1);
            break;
          default:
            parseError = true;
        }
        
        if (!parseError) {
          receivedDirection[sizeof(receivedDirection) - 1] = '\0';
          newCommandReceived = true;
          
          if (printDebug) {
            Serial.printf("Received: angle=%d, direction=%s, speed=%d\n", 
                        receivedAngle, receivedDirection, receivedSpeed);
          }
        }
      }
    }
  }

  // receivedAngle = 0;
  // receivedSpeed = 50;
  // strncpy((char*)receivedDirection, "LEFT", sizeof(receivedDirection) - 1);
  // receivedDirection[sizeof(receivedDirection) - 1] = '\0';
  // newCommandReceived = true;


  // Handle periodic tasks
  unsigned long currentTime = millis();

  if (healthPoints <= 0) {
    stopCar();
    return;
  }
  // Update RPM and control signals at intervals
  if (currentTime - prevRpmCalcTime > rpmCalcInterval) {
    rpmCalculation();
    
    // Set left and right pulse count to 0
    
    leftPulseCount = 0;
    rightPulseCount = 0;

    updateControlSignals();
    prevRpmCalcTime = currentTime;
  }

  // Process received steering commands only in autonomous mode
  if (autonomousMode && newCommandReceived) {
    steer(receivedAngle, receivedDirection, receivedSpeed);
    newCommandReceived = false;
  }

  // Prepare and send motor signals
  int controlled_left_pwm, controlled_right_pwm;
  prepareControlledMotorSignals(desiredLeftPWM, desiredRightPWM, controlled_left_pwm, controlled_right_pwm);
  if (printDebug) {
    Serial.printf("Control signals: Left: %d, Right: %d\n", controlSignalLeft, controlSignalRight);
    Serial.printf("Controlled Left PWM: %d, Controlled Right PWM: %d\n", controlled_left_pwm, controlled_right_pwm);
  }
  sendMotorSignals(controlled_left_pwm, desiredLeftDirection, controlled_right_pwm, desiredRightDirection);

  // Replace servo handling code with handleServo function
  handleServo(servoOff, swingServo, swingSpeed);
}

// Function to stop the car
void stopCar() {
  // ...code to stop the car...
  ledcWrite(pwmPinLeft, 0);
  ledcWrite(pwmPinRight, 0);
}

// Function to move forward
void moveForward() {
  sendMotorSignals(defaultPWM, LOW, defaultPWM, LOW);  // Set direction to LOW for forward
}

// Function to turn left
void turnLeft() {
  sendMotorSignals(defaultPWM / 2, LOW, defaultPWM, LOW);
}

// Function to turn right
void turnRight() {
  sendMotorSignals(defaultPWM, LOW, defaultPWM / 2, LOW);
}

// Function to prepare the motor signals based on the linear and angular velocity
void prepareIdealMotorSignals(
  float linear_velocity, 
  float angular_velocity, 
  int& left_pwm,
  int& left_direction,
  int& right_pwm,
  int& right_direction
) {
  // Calculate the angular velocity of each wheel
  float omega_l = (linear_velocity - angular_velocity * WHEEL_BASE / 2) / WHEEL_RADIUS;
  float omega_r = (linear_velocity + angular_velocity * WHEEL_BASE / 2) / WHEEL_RADIUS;

  // Clip the angular velocity to the maximum limits
  if (omega_l > MAX_WHEEL_VELOCTY) {
    omega_l = MAX_WHEEL_VELOCTY;
  } else if (omega_l < -MAX_WHEEL_VELOCTY) {
    omega_l = -MAX_WHEEL_VELOCTY;
  }
  if (omega_r > MAX_WHEEL_VELOCTY) {
    omega_r = MAX_WHEEL_VELOCTY;
  } else if (omega_r < -MAX_WHEEL_VELOCTY) {
    omega_r = -MAX_WHEEL_VELOCTY;
  }

  // Convert the angular velocity to PWM signals
  convertAngularVelocityToPWM(omega_l, left_pwm, left_direction);
  convertAngularVelocityToPWM(omega_r, right_pwm, right_direction);

  // Clip to the limits of PWM
  left_pwm = min(max(left_pwm, 0), LEDC_RESOLUTION);
  right_pwm = min(max(right_pwm, 0), LEDC_RESOLUTION);
}

// Function to prepare motor signals after incoporating control signals
void prepareControlledMotorSignals(
  int ideal_left_pwm, 
  int ideal_right_pwm,
  int& controlled_left_pwm,
  int& controlled_right_pwm
  ) {
  // Incorporate control signals if needed
  controlled_left_pwm = ideal_left_pwm;
  controlled_right_pwm = ideal_right_pwm;
  if (ENABLE_CONTROL) {
    controlled_left_pwm += controlSignalLeft;
    controlled_right_pwm += controlSignalRight;
  }

  // Clip to the limits of PWM
  controlled_left_pwm = min(max(controlled_left_pwm, 0), LEDC_RESOLUTION);
  controlled_right_pwm = min(max(controlled_right_pwm, 0), LEDC_RESOLUTION);
}

// Adjust convertAngularVelocityToPWM() to match controller.ino
void convertAngularVelocityToPWM(float omega, int& pwm_ref, int& direction) {
  if (omega > 0.0) {
    direction = LOW;  // LOW for forward
    pwm_ref = (int)(mapf(omega, 0, MAX_WHEEL_VELOCTY, 0, (float)LEDC_RESOLUTION));
  } else if (omega < 0.0) {
    direction = HIGH; // HIGH for backward
    pwm_ref = (int)(mapf(-omega, 0, MAX_WHEEL_VELOCTY, 0, (float)LEDC_RESOLUTION));
  } else {
    // When omega is zero, maintain the last direction
    pwm_ref = 0;
  }
}

// Function to map a value from one range to another in floating point
float mapf(float value, float inMin, float inMax, float outMin, float outMax) {
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// Remove inversion of motor directions in sendMotorSignals()
void sendMotorSignals(
  int left_pwm, 
  int left_direction, 
  int right_pwm, 
  int right_direction
) {
  // Set the direction of the motors - we need to be careful with the direction.
#if EXTRA_CARE
  // Extra care is needed to stop the motor before switching the direction.
  // If the previous direction is different from the current direction, we need to stop the motor first.
  if (left_direction != desiredLeftDirection) {
    ledcWrite(pwmPinLeft, 0);
  }
  if (right_direction != desiredRightDirection) {
    ledcWrite(pwmPinRight, 0);
  }
  delay(MOTOR_STOP_DELAY);
#endif

  // Set the direction of the motors
  digitalWrite(dirPinLeft, left_direction);
  digitalWrite(dirPinRight, right_direction);



  // Print pwms
  if (printDebug) {
    Serial.printf("Actual Left PWM: %d, Actual Right PWM: %d\n", left_pwm, right_pwm);
  }

  // Set the PWM signals for the motors
  ledcWrite(pwmPinLeft, left_pwm);
  ledcWrite(pwmPinRight, right_pwm);
}

void rpmCalculation() {
  leftRPM = (abs(leftPulseCount) * 60000) / ((pulsesPerRevolution) * (millis() - prevRpmCalcTime));
  rightRPM = (abs(rightPulseCount) * 60000) / ((pulsesPerRevolution) * (millis() - prevRpmCalcTime));

  leftRPM = (leftRPM * 2) / gearRatio;
  rightRPM = (rightRPM * 2) / gearRatio;
}

void updateControlSignals() {
  // Calculate the desired PWM for the left and right motors
  int current_pwm_left = map(leftRPM, 0, MAX_RPM, 0, LEDC_RESOLUTION);
  int current_pwm_right = map(rightRPM, 0, MAX_RPM, 0, LEDC_RESOLUTION);

  if (printDebug) {
    Serial.printf("Current RPM - Left: %d (direction: %d), Right: %d (direction: %d)\n", leftRPM, leftDirection, rightRPM, rightDirection);
    Serial.printf("Current PWM - Left: %d, Right: %d\n", current_pwm_left, current_pwm_right);
  }

  // PID control
  controlSignalLeft = leftPID.compute(desiredLeftPWM, current_pwm_left);
  controlSignalRight = rightPID.compute(desiredRightPWM, current_pwm_right);
}

// Purpose: update the direction and pulse count of the encoder based on the current encoder values
// Inputs: encoder_A_val, encoder_B_val, last_encoded_val, direction, pulse_count
// Outputs: last_encoded_val, direction, pulse_count
void readEncoderValue(
    int encoder_A_val,
    int encoder_B_val,
    int& last_encoded_val,
    int& direction,
    int& pulse_count
) {
  int MSB = encoder_A_val;
  int LSB = encoder_B_val;

  // Combine the two bits into a single integer for easier comparison
  int encoded = (MSB << 1) | LSB;
  
  // Compute the direction based on the changes in the encoder states
  int sum = (last_encoded_val << 2) | encoded;  // Combine previous and current state

  // Update the encoder position based on the direction
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    direction = 1;  // Clockwise
    pulse_count++;
  } else if (sum == 0b1110 || sum == 0b1000 || sum == 0b0001 || sum == 0b0111) {
    direction = -1;  // Counterclockwise
    pulse_count--;
  }

  // Update the last encoded state
  last_encoded_val = encoded;

  return;
}

// Implement the steer function
/**
 * Processes steering commands for autonomous mode
 * @param angle: Turning angle (0-45 degrees)
 * @param direction: "LEFT", "RIGHT", or "FORWARD"
 * @param speed: Desired speed (0-100)
 */
void steer(int angle, const char* direction, int speed) {
  float maxSteeringAngle = 50.0;  // Use MAX_STEERING_ANGLE from wall_follow.h
  float moveSpeed = (speed / 100.0) * MAX_LINEAR_VELOCTY;
  float angular_velocity = (angle / maxSteeringAngle) * MAX_ANGULAR_VELOCITY;
  Serial.print("moveSpeed: ");
  Serial.println(moveSpeed);
  Serial.print("angular_velocity: ");
  Serial.println(angular_velocity);

  if (strcmp(direction, "LEFT") == 0) {
    // Positive angular velocity for left turn
    angular_velocity = angular_velocity;
  } else if (strcmp(direction, "RIGHT") == 0) {
    // Negative angular velocity for right turn
    angular_velocity = -angular_velocity;
  } else {
    // No turning
    angular_velocity = 0;
  }

  int left_pwm, left_direction, right_pwm, right_direction;
  prepareIdealMotorSignals(moveSpeed, angular_velocity, left_pwm, left_direction, right_pwm, right_direction);
  if (printDebug) {
    Serial.printf("Steering: angle=%d, direction=%s, speed=%d\n", angle, direction, speed);
    Serial.printf("Desired Left: PWM=%d, Direction=%d\n", left_pwm, left_direction);
    Serial.printf("Desired Right: PWM=%d, Direction=%d\n", right_pwm, right_direction);
  }
  desiredLeftPWM = left_pwm;
  desiredLeftDirection = left_direction;
  desiredRightPWM = right_pwm;
  desiredRightDirection = right_direction;
}

// Add new handler function
/**
 * Handles mode switching between autonomous and manual control
 * Updates autonomousMode flag and responds to client
 */
// void handleSetMode() {
//   if (server.hasArg("mode")) {
//     String mode = server.arg("mode");
//     autonomousMode = (mode == "autonomous");
//     server.send(200, "text/plain", "Mode updated");
//     Serial.printf("Mode updated to: %s\n", autonomousMode ? "autonomous" : "manual");
//   } else {
//     server.send(400, "text/plain", "Bad Request");
//   }
// }

// // Set motor signals based on the desired PWM values
// // Function to handle the motor control request
// void handleSetMotor() {
//   if (server.hasArg("speed") && server.hasArg("forwardBackward") && server.hasArg("turnRate")) {

//     float motor_speed = server.arg("speed").toFloat(); // percent from 0 to 100
//     String forward_backward = server.arg("forwardBackward"); // "Forward" or "Backward"
//     float turn_rate = server.arg("turnRate").toFloat(); // percent from -50 to 50

//     // Use these parameters to call steer appropriately
//     motor_speed = motor_speed * (forward_backward == "Forward" ? 1 : -1);
//     if (turn_rate < 0) {
//       steer((int)-turn_rate, "RIGHT", motor_speed);
//     } else {
//       steer((int)turn_rate, "LEFT", motor_speed);
//     }
//   }
// }

// void handleSetServo() {
//   if (server.hasArg("servo")) {
//     String servoState = server.arg("servo");
//     if (servoState == "off") {
//       servoOff = true;
//       swingServo = false;
//     } else if (servoState == "on") {
//       servoOff = false;
//       swingServo = true;
//     }
//   }
//   server.send(200, "text/plain", "Servo parameters updated");
// }

// Function to map angle to duty cycle
unsigned int angleToDuty(int angle) {
    return map(angle, servoMinAngle, servoMaxAngle, servoMinDuty, servoMaxDuty);
}

// Function to set servo angle
void setServoAngle(int angle) {
    // Limit angle between servoMinAngle and servoMaxAngle
    if (angle < servoMinAngle) angle = servoMinAngle;
    if (angle > servoMaxAngle) angle = servoMaxAngle;
    unsigned int duty = angleToDuty(angle);
    ledcWrite(servoPWMPin, duty);
}

// Function to swing the servo between angles
void swingServoFunction(int swingSpeed) {
    static int servoAngle = servoDefaultAngle - servoSwingAngle;
    static int servoIncrement = swingSpeed; // Use swingSpeed for increment

    setServoAngle(servoAngle);

    servoAngle += servoIncrement;
    if (servoAngle >= servoDefaultAngle + servoSwingAngle || servoAngle <= servoDefaultAngle - servoSwingAngle) {
        servoIncrement = -servoIncrement; // Reverse direction
    }
}

// Add handleServo function
void handleServo(bool servoOff, bool swingServo, int swingSpeed) {
    if (servoOff) {
        setServoAngle(servoMinAngle); // Servo is OFF, set to 0 degrees
    } else if (swingServo) {
        swingServoFunction(swingSpeed); // Swing servo
    } else {
        setServoAngle(servoDefaultAngle); // Servo is ON, set to default angle
    }
}
