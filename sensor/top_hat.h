#ifndef TOP_HAT_H
#define TOP_HAT_H

#include <Wire.h>

#define TOP_HAT_I2C_ADDRESS 0x28
#define TOP_HAT_SDA 10
#define TOP_HAT_SCL 0
#define TOP_HAT_I2C_FREQUENCY 40000

TwoWire WireTH = TwoWire(1); 

void initTopHat();
uint8_t readTopHatData();


void initTopHat() {
  // Separate I2C bus for the top hat
  // Using 10 as SDA and 0 as SCL
  WireTH.begin(TOP_HAT_SDA, TOP_HAT_SCL, TOP_HAT_I2C_FREQUENCY);
}

void sendTopHatData(uint8_t packet) {
  WireTH.beginTransmission(TOP_HAT_I2C_ADDRESS);
  WireTH.write(packet);
  WireTH.endTransmission();
}

uint8_t readTopHatData() {
  uint8_t dataReceived = WireTH.requestFrom(TOP_HAT_I2C_ADDRESS, 1);
  uint8_t data = 0;
  if (dataReceived > 0) {
      Serial.print("\nReceived from slave: ");
      while (WireTH.available()) {
          data = WireTH.read();
          Serial.print((int)data);
      }
  } else {
        Serial.println("No data received from slave");
    }
  return (int)data;
}

#endif
