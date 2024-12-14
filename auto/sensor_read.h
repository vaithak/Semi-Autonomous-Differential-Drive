#ifndef SENSOR_READ_H
#define SENSOR_READ_H

#include <Wire.h>

#define I2C_SLAVE_ADDR 0x35
#define SDA_PIN 25
#define SCL_PIN 0

#define I2C_PRINT_FREQUENCY 1000
uint32_t last_i2c_print_time = 0;

// Buffer for receiving data
volatile uint8_t receivedData[32];
volatile uint8_t steerDataLength = 0;
volatile bool steerDataReceived = false;

// Variables to store the decoded speed, angle and direction
// from the received data
volatile uint8_t receivedSpeed = 0;
volatile uint8_t receivedAngle = 0;
volatile char receivedDirection = 'F'; // F - Forward, L - Left, R - Right
volatile uint8_t wifiPackets = 0;

// Buffer for sending data
uint8_t sendData[32];
int count = 0;

TwoWire WireSensor = TwoWire(0); 

// Write the code to decode the received data
// 1st byte: Speed (0-100)
// Next 1 byte: Angle (0-50)
// Next 1 byte: Direction (F - Forward, L - Left, R - Right)
// Next 1 byte: Number of used wifi packets
void decodeDataFromI2C() {
  if (steerDataReceived) {
    receivedSpeed = receivedData[0];
    receivedAngle = receivedData[1];
    receivedDirection = receivedData[2];
    wifiPackets += receivedData[3]; // cumulative wifi packets
    steerDataReceived = false;
  }
}

// Called when the master sends data
void receiveEvent(int bytesin) {
  uint8_t len = 0;

  while (WireSensor.available()) {
    if (len < sizeof(receivedData)) {
      receivedData[len++] = WireSensor.read();
    } else {
      WireSensor.read();  // Discard excess data
    }
  }
  steerDataLength = len;
  steerDataReceived = true;

  // Decode the received data
  decodeDataFromI2C();
}

// Called when the master requests data
void sendCount() {
  WireSensor.write(count++);
}

void sensor_i2c_setup() {
  // put your setup code here, to run once:
  WireSensor.begin((uint8_t)I2C_SLAVE_ADDR, SDA_PIN, SCL_PIN, 40000);

  // Register event handlers
  WireSensor.onReceive(receiveEvent);
  WireSensor.onRequest(sendCount);

  if (millis() - last_i2c_print_time > I2C_PRINT_FREQUENCY) {
    Serial.println("ESP32 I2C Slave initialized");
    Serial.printf("Address: 0x%02X, SDA: %d, SCL: %d\n", I2C_SLAVE_ADDR, SDA_PIN, SCL_PIN);

    last_i2c_print_time = millis();
  }
}

#endif // SENSOR_READ_H

