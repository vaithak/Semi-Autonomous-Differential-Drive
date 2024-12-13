#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFiUdp.h>     
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <ArduinoJson.h>
#include "planning.h"
#include "web.h"
#include "mode_select_web.h"
#include "rgb.h"
#include "top_hat.h"

const char* ssid = "GM Lab Public WIFI";   
const char* password = "";             

IPAddress local_IP(192, 168, 1, 105);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0); 

// IP and port of the auto.ino board
const char* udpAddress = "192.168.1.104";  // IP address of auto.ino
const unsigned int udpPort = 8888;         // UDP port to send data to

WiFiUDP udp;

WebServer server(80);
Planner planner;

WebSocketsServer webSocket = WebSocketsServer(81);  // Initialize WebSocket server

// Function prototypes
void sendSteeringCommand(int angle, const char* direction, int speed);
void handleRoot();
void handleData();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

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
  WiFi.mode(WIFI_MODE_STA);
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(ssid, password);

  // Initialize web server routes regardless of Wi-Fi connection status
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/setMode", handleSetMode);

  // Start the web server
  server.begin();
  Serial.println("Web server started");

  // Initialize WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started");

  // Start UDP
  udp.begin(udpPort);
  Serial.print("UDP client started on port ");
  Serial.println(udpPort);

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
  webSocket.loop();

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
    planner.planLogic();  // Ensure this is called regularly

    delay(80);
    handleRGB();
    // Print sensor values every 1 second
    // if (millis() - lastPrint > 1000) {
    //   int d_front, d_left, d_right;
    //   readToFSensors(d_front, d_left, d_right);
    //   // Serial.printf("Sensors - Front: %d mm, Left: %d mm, Right: %d mm\n", d_front, d_left, d_right);
    //   lastPrint = millis();
    // }

    // Send sensor data over WebSocket
    static unsigned long lastBroadcast = 0;
    unsigned long lastRead = 0;
    if (millis() - lastBroadcast > 100) {   // Broadcast every 100ms
      int d_front, d_left, d_right;
      int steering_angle, speed;
      const char* direction;
      readToFSensors(d_front, d_left, d_right);
      readSteeringResult(steering_angle, direction, speed);
      RobotState currentState = planner.getCurrentState();
      StaticJsonDocument<200> doc;
      doc["front"] = d_front;
      doc["left"] = d_left;
      doc["right"] = d_right;
      doc["angle"] = steering_angle;
      doc["direction"] = direction;
      doc["speed"] = speed;
      doc["x"] = currentState.x;
      doc["y"] = currentState.y;
      doc["theta"] = currentState.theta;
      String jsonString;
      serializeJson(doc, jsonString);
      webSocket.broadcastTXT(jsonString);
      lastBroadcast = millis();
    }

    // Read top hat data every 500ms
    if (millis() - lastRead > 500) {
      uint8_t topHatData = readTopHatData();
      // send top hat data over UDP as json
      if (udp.beginPacket(udpAddress, udpPort)) {
        StaticJsonDocument<200> doc;
        doc["HP"] = topHatData;
        char jsonString[200];
        serializeJson(doc, jsonString);
        udp.write((uint8_t*)jsonString, strlen(jsonString));
        udp.endPacket();
      }
      lastRead = millis();
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
  StaticJsonDocument<200> doc;
  doc["angle"] = angle;
  doc["direction"] = direction;
  doc["speed"] = speed;  // Add speed to JSON
  char jsonString[200];
  serializeJson(doc, jsonString);

  // Send UDP packet with proper error handling
  if(udp.beginPacket(udpAddress, udpPort)) {
    udp.write((uint8_t*)jsonString, strlen(jsonString));
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
void handleData() {
  int d_front, d_left, d_right;
  int steering_angle, speed;
  float x, y, theta;
  const char* direction;
  readToFSensors(d_front, d_left, d_right);
  readSteeringResult(steering_angle, direction, speed);
  RobotState currentState = planner.getCurrentState();
  x = currentState.x;
  y = currentState.y;
  theta = currentState.theta;
  // Prepare JSON data
  String jsonString = "{";
  jsonString += "\"front\":" + String(d_front) + ",";
  jsonString += "\"left\":" + String(d_left) + ",";
  jsonString += "\"right\":" + String(d_right);
  jsonString += "\"angle\":" + String(steering_angle) + ",";
  jsonString += "\"direction\":\"" + String(direction) + "\",";
  jsonString += "\"speed\":" + String(speed) + ",";
  jsonString += "\"x\":" + String(planner.getCurrentState().x) + ",";
  jsonString += "\"y\":" + String(planner.getCurrentState().y) + ",";
  jsonString += "\"theta\":" + String(planner.getCurrentState().theta);
  jsonString += "}";

  // Send JSON data
  server.send(200, "application/json", jsonString);
}

// Implement handleRoot function similar to auto.ino
void handleRoot() {
  // Log that the root page is requested
  Serial.println("Root page requested");
  server.send_P(200, "text/html", WEBPAGE);
}

// Function to receive mode from web interface
void handleSetMode() {
  Serial.println("Request for setting mode received");

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
  } else {
    // Send the HTML page
    server.send_P(200, "text/html", MODE_SELECT_PAGE);
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("[%u] Connected from %s\n", num, ip.toString().c_str());
      break;
    }
    case WStype_TEXT:
      Serial.printf("[%u] Received text: %s\n", num, payload);
      // Handle incoming messages if needed
      break;
    default:
      break;  
  }  
}  // Close the function properly