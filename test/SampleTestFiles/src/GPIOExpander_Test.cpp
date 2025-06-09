//This code is used to cycle all of our GPIO expander pins from low to high and vice versa every second. Using the TCA9539 GPIO expanders
#include "Arduino_PortentaBreakout.h"
#include <arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_H3LIS331.h>
#include <math.h>
#include <serial.h>
#include "SDMMCBlockDevice.h"
#include "FATFileSystem.h"
#include "SocketHelpers.h"
#include <stdio.h>

// I²C addresses of the two TCA9539 expanders
const uint8_t TCA1 = 0x74;
const uint8_t TCA2 = 0x76;

// TCA9539 registers
constexpr uint8_t REG_OUTPUT_PORT0   = 0x02;
constexpr uint8_t REG_OUTPUT_PORT1   = 0x03;
constexpr uint8_t REG_CONFIG_PORT0   = 0x06;
constexpr uint8_t REG_CONFIG_PORT1   = 0x07;

void writeRegister16(uint8_t addr, uint8_t reg, uint16_t value) {
  Wire.beginTransmission(addr);     // start talking to expander
  Wire.write(reg);                  // point at the first register
  Wire.write(value & 0xFF);         // low byte → Port0
  Wire.write((value >> 8) & 0xFF);  // high byte → Port1
  Wire.endTransmission();           // send it
}

void setup() {
  Wire.begin();  // join I2C bus
  // configure all pins on both expanders as outputs (0 = output)
  writeRegister16(TCA1, REG_CONFIG_PORT0, 0x0000);
  writeRegister16(TCA2, REG_CONFIG_PORT0, 0x0000);
}

void loop() {
  // drive all pins HIGH (1 → high output)
  writeRegister16(TCA1, REG_OUTPUT_PORT0, 0xFFFF);
  writeRegister16(TCA2, REG_OUTPUT_PORT0, 0xFFFF);
  delay(1000);

  // drive all pins LOW
  writeRegister16(TCA1, REG_OUTPUT_PORT0, 0x0000);
  writeRegister16(TCA2, REG_OUTPUT_PORT0, 0x0000);
  delay(1000);
}