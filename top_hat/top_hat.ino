/* 
Sample code for the Top Hat I2C slave device. 
*/

#include <Wire.h>

#define TOP_HAT_I2C_ADDRESS 0x28  // Slave device 


int packet = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(19, 4, 40000);
  Serial.print("Starting...");
}

void loop() {
  packet += 1;
  Wire.beginTransmission(TOP_HAT_I2C_ADDRESS);
  Wire.write(packet);
  uint8_t error = Wire.endTransmission(true);
  if (error == 0) {
      Serial.println("Data sent successfully");
  } else {
      Serial.printf("Error sending data: %d\n", error);
  }
  uint8_t bytesReceived = Wire.requestFrom(TOP_HAT_I2C_ADDRESS, 1);
  uint8_t byteIn = 0;
  Serial.println(bytesReceived);
  if (bytesReceived > 0) {
      Serial.print("Received from slave: ");
      while (Wire.available()) {
          byteIn = Wire.read();
          Serial.printf("0x%02X \n", byteIn);
      }
  } else {
        Serial.println("No data received from slave");
    }
  delay(500);
}

