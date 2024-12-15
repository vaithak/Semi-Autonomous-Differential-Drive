#include <WiFi.h>
#include <WebServer.h>
#include <WiFiUdp.h>     
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "planning.h"
#include "web.h"
// #include "mode_select_web.h"
#include "rgb.h"
#include "top_hat.h"

const char* ssid = "GM Lab Public WIFI";  // Wi-Fi network name
const char* password = "";   // Wi-Fi password (empty for no password)    

IPAddress local_IP(192, 168, 1, 105);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0); 

// IP and port of the auto.ino board
const char* udpAddress = "192.168.1.104";  // IP address of auto.ino
const unsigned int udpPort = 8888;         // UDP port to send data to

WiFiUDP udp;

WebServer server(80);
Planner planner;
bool manual_mode = false; // Default to autonomous mode

// Function prototypes
void sendSteeringCommand(int angle, const char* direction, int speed);
void handleRoot();
// void handleData();
void handleManualSwitch();
void handleSetMode();
void handleSetMotor();
void handleSetServo();

// Store number of wifi packets received
uint8_t wifiPacketCount = 0;
uint8_t healthPoints = 100;

// Store servo state
bool servoOff = false;

// Store requested motor speed and direction
int requestedSpeed = 0;
int requestedTurnRate = 0;
String forward_backward = "Forward";

// Last top hat read
uint8_t lastTopHatRead = 0;

void setup() {
  setupRGB();
  Serial.begin(115200);
  delay(1000);  // Add delay to ensure serial monitor is ready
  Serial.println("\n\nStarting Sensor Node...");


  // Initialize I2C with custom SDA and SCL pins
  Wire.begin(SDA_PIN, SCL_PIN);
  // Initialize the planner
  planner.setup();
  initTopHat();

  // Initialize the top hat sensor


  // Wi-Fi setup as STA mode
  WiFi.mode(WIFI_MODE_STA);
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(ssid, password, 11);

  Serial.print("Waiting for WiFi connection");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("connected to %s on ", ssid);
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Failed to connect to WiFi!");
  }

  // Initialize web server routes regardless of Wi-Fi connection status
  server.on("/", handleRoot);
  // server.on("/data", handleData);
  server.on("/manual", handleManualSwitch);
  server.on("/setMode", handleSetMode);
  server.on("/setMotor", handleSetMotor);
  server.on("/setServo", handleSetServo);

  // Start the web server
  server.begin();
  Serial.println("Web server started");

  // Start UDP
  udp.begin(udpPort);
  Serial.print("UDP client started on port ");
  Serial.println(udpPort);
}

/**
 * Main loop handles:
 * 1. Web server and WebSocket communication
 * 2. Wall following logic
 * 3. Sensor data broadcasting
 */
void loop() {
  static unsigned long lastPrint = 0;

  // Always handle client requests
  server.handleClient();
  // Handle Wi-Fi connection
  if (WiFi.status() != WL_CONNECTED) {
    // Try to reconnect to Wi-Fi if connection is lost
    static unsigned long lastReconnectAttempt = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastReconnectAttempt >= 5000) {  // Try every 5 seconds
      Serial.println("Reconnecting to WiFi...");
      WiFi.reconnect();
      lastReconnectAttempt = currentTime;
    }
  } else {
    if (manual_mode) {
      if (requestedTurnRate < 0) {
        sendSteeringCommand((int)-requestedTurnRate, "RIGHT", requestedSpeed * (forward_backward == "Forward" ? 1 : -1));
      } else {
        sendSteeringCommand((int)requestedTurnRate, "LEFT", requestedSpeed * (forward_backward == "Forward" ? 1 : -1));
      }
    } else {
      planner.planLogic();  // Ensure this is called regularly
    }
    delay(50);

    handleRGB();
    // Print sensor values every 1 second
    // if (millis() - lastPrint > 1000) {
    //   int d_front, d_left, d_right;
    //   readToFSensors(d_front, d_left, d_right);
    //   // Serial.printf("Sensors - Front: %d mm, Left: %d mm, Right: %d mm\n", d_front, d_left, d_right);
    //   lastPrint = millis();
    // }

    // Read top hat data every 500ms
    if (millis() - lastTopHatRead > 500) {
      healthPoints = readTopHatData();
      // Send wifi packet to top hat
      sendTopHatData(wifiPacketCount);
      wifiPacketCount = 0;
      lastTopHatRead = millis();
    }
    
    // TODO: Send UDP Packet of current HP to auto.ino
  }
}
// Top hat data: 0
// Top hat data: 0
// Left wall follow mode
// Top hat data: 0
// Vive 1: 2654, 4308 | Vive 2: 2823, 2825 

// Vive 1: 1739, 4338 | Vive 2: 1563, 4343


//Left wall follow mode
//Front: 172
//Left: 342
//Right: OOR
//Turning left - steering angle: 50.00
//Top hat data: 0
//Vive 1: 2809, 2994 | Vive 2: 2769, 2820 

/**
 * Sends steering commands to auto.ino via UDP
 * @param angle: Desired turning angle
 * @param direction: "LEFT", "RIGHT", or "FORWARD"
 * @param speed: Desired speed
 */
void sendSteeringCommand(int angle, const char* direction, int speed) {
  // StaticJsonDocument<200> doc;
  // doc["angle"] = angle;
  // doc["direction"] = direction;
  // doc["speed"] = speed;  // Add speed to JSON
  // char jsonString[200];
  // serializeJson(doc, jsonString);
  
  // send UDP packet as bytes
  uint8_t sendData[5];
  sendData[0] = (uint8_t)(speed + 100); // mapping -100 to 100 -> 0 to 200
  sendData[1] = (uint8_t)angle;
  sendData[2] = direction[0];
  sendData[3] = healthPoints;
  sendData[4] = servoOff ? 0 : 1;

  // Send UDP packet with proper error handling
  if(udp.beginPacket(udpAddress, udpPort)) {
    udp.write(sendData, 5);  // Use fixed size 5 instead of strlen
    if(udp.endPacket()) {
      // Serial.printf("[Successful] Sent packet: %s\n", jsonString);
    } else {
      Serial.println("Failed to send UDP packet");
    }
  } else {
    Serial.println("Could not begin UDP packet");
  }
}

// Function to send sensor data to clients
// void handleData() {
//   int d_front, d_left, d_right;
//   int steering_angle, speed;
//   float x, y, theta;
//   const char* direction;
//   readToFSensors(d_front, d_left, d_right);
//   readSteeringResult(steering_angle, direction, speed);
//   RobotState currentState = planner.getCurrentState();
//   x = currentState.x;
//   y = currentState.y;
//   theta = currentState.theta;
//   // Prepare JSON data
//   String jsonString = "{";
//   jsonString += "\"front\":" + String(d_front) + ",";
//   jsonString += "\"left\":" + String(d_left) + ",";
//   jsonString += "\"right\":" + String(d_right);
//   jsonString += "\"angle\":" + String(steering_angle) + ",";
//   jsonString += "\"direction\":\"" + String(direction) + "\",";
//   jsonString += "\"speed\":" + String(speed) + ",";
//   jsonString += "\"x\":" + String(planner.getCurrentState().x) + ",";
//   jsonString += "\"y\":" + String(planner.getCurrentState().y) + ",";
//   jsonString += "\"theta\":" + String(planner.getCurrentState().theta);
//   jsonString += "}";

//   // Send JSON data
//   server.send(200, "application/json", jsonString);
// }

// Implement handleRoot function similar to auto.ino
void handleRoot() {
  // Log that the root page is requested
  Serial.println("Root page requested");
  server.send_P(200, "text/html", WEBPAGE);
}

// Set motor signals based on the desired PWM values
// Function to handle the motor control request
void handleSetMotor() {
  if (server.hasArg("speed") && server.hasArg("forwardBackward") && server.hasArg("turnRate")) {
    requestedSpeed = server.arg("speed").toInt(); // percent from 0 to 100
    forward_backward = server.arg("forwardBackward"); // "Forward" or "Backward"
    requestedTurnRate = server.arg("turnRate").toInt(); // percent from -50 to 50
    server.send(200, "text/plain", "Motor parameters updated");
    wifiPacketCount++;
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

void handleSetServo() {
 if (server.hasArg("servo")) {
    String servoState = server.arg("servo");
    if (servoState == "Off") {
      servoOff = true;
    } else if (servoState == "On") {
      servoOff = false;
    }
  }
  server.send(200, "text/plain", "Servo parameters updated");
}

void handleManualSwitch() {
  // Log that the manual switch is requested
  Serial.println("Manual switch requested");
  if (server.hasArg("bool")) {
    manual_mode = server.arg("bool") == "true";
    server.send(200, "text/plain", "Manual mode updated");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

// Function to receive mode from web interface
void handleSetMode() {
  Serial.println("Request for setting mode received");
  wifiPacketCount++;

  if (server.hasArg("mode")) {
    // mode is a string that can be among
    // "leftWallFollow", "rightWallFollow",
    // "attackRampBlueTower", "attackRampRedTower",
    // "attackGroundNexusRight", "attackGroundNexusLeft",
    // "attackBlueGroundNexusCenter", "attackRedGroundNexusCenter",
    // "gridMode" - for attacking TA Bot
    const char* mode = server.arg("mode").c_str();
    if (strcmp(mode, "gridMode") == 0) {
      // Extract x and y coordinates
      if (server.hasArg("x") && server.hasArg("y")) {
        int x = server.arg("x").toInt();
        int y = server.arg("y").toInt();
        planner.setWaypointsAndMode(x, y, mode);
      }
    } else {
      planner.setWaypointsAndMode(0, 0, mode);
    }
  }
  // else {
  //   // Send the HTML page
  //   server.send_P(200, "text/html", MODE_SELECT_PAGE);
  // }
}
