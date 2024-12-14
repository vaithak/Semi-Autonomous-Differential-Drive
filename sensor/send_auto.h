#ifndef SEND_AUTO_H
#define SEND_AUTO_H

#include <Wire.h>

#define AUTO_I2C_SLAVE_ADDR 0x35
#define SENSOR_AUTO_SDA_PIN 10
#define SENSOR_AUTO_SCL_PIN 0

TwoWire WireAuto = TwoWire(1);

void initSensorAuto() {
  // Separate I2C bus for the top hat
  // Using 10 as SDA and 0 as SCL
  WireAuto.begin(SENSOR_AUTO_SDA_PIN, SENSOR_AUTO_SCL_PIN);
}

void sendAutoData(uint8_t data[32], int length) {
  WireAuto.beginTransmission(AUTO_I2C_SLAVE_ADDR);
  WireAuto.write(data, length);
  WireAuto.endTransmission();
}

#endif
