#ifndef TOP_HAT_H
#define TOP_HAT_H

#include <Wire.h>

#define TOP_HAT_I2C_ADDRESS 0x28  // Slave device 

// Function declarations
void initTopHat();
uint8_t readTopHatData();

void initTopHat() {
  // Check if the device acknowledges on the I2C bus
  Wire.beginTransmission(TOP_HAT_I2C_ADDRESS);
  if (Wire.endTransmission() == 0) {
    Serial.print("Top Hat device found at address 0x");
    Serial.println(TOP_HAT_I2C_ADDRESS, HEX);
  } else {
    Serial.print("Top Hat device not found at address 0x");
    Serial.println(TOP_HAT_I2C_ADDRESS, HEX);
  }
}

uint8_t readTopHatData() {
  uint8_t data = 0;
  Wire.requestFrom(TOP_HAT_I2C_ADDRESS, (uint8_t)1);
  if (Wire.available()) {
    data = Wire.read();
  }
  return data;
}

#endif // TOP_HAT_H
