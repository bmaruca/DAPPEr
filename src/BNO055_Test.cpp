#include "Arduino_PortentaBreakout.h"
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "SDMMCBlockDevice.h"
#include "FATFileSystem.h"
#include <stdio.h>

//---------------PIN & REGISTER DEFINITIONS---------------
#define LAST_ARDUINO_PIN_NUMBER LEDB + 1

// BNO055 I²C address (low by default; tie ADR high for 0x29)
static const uint8_t BNO_ADDRESS   = 0x28;

// BNO055 registers (page 0 unless noted)
static const uint8_t REG_PAGE_ID     = 0x07;
static const uint8_t REG_CHIP_ID     = 0x00;
static const uint8_t REG_PWR_MODE    = 0x3E;
static const uint8_t REG_OPR_MODE    = 0x3D;
static const uint8_t REG_SYS_TRIGGER = 0x3F;
static const uint8_t REG_UNIT_SEL    = 0x3B;

// Data output registers (LSB first)
static const uint8_t REG_ACCEL_DATA = 0x08;  // 6 bytes: AX_L, AX_H, AY_L, AY_H, AZ_L, AZ_H
static const uint8_t REG_MAG_DATA   = 0x0E;  // 6 bytes
static const uint8_t REG_GYRO_DATA  = 0x14;  // 6 bytes
static const uint8_t REG_TEMP       = 0x34;  // 1 byte

// Operation modes
static const uint8_t MODE_CONFIG = 0x00;
static const uint8_t MODE_NDOF   = 0x0C;

//---------------7-SEGMENT DISPLAY SETUP---------------
breakoutPin A  = PWM8;
breakoutPin B  = PWM9;
breakoutPin C  = PWM7;
breakoutPin D  = PWM3;
breakoutPin E  = PWM4;
breakoutPin Z  = PWM5;
breakoutPin G  = PWM6;
breakoutPin DP = PWM2;

int seg[]{ A, B, C, D, E, Z, G, DP };
byte Chars[17][7] = {
  {1,1,1,1,1,1,0},  //0
  {0,1,1,0,0,0,0},  //1
  {1,1,0,1,1,0,1},  //2
  {1,1,1,1,0,0,1},  //3
  {0,1,1,0,0,1,1},  //4
  {1,0,1,1,0,1,1},  //5
  {1,0,1,1,1,1,1},  //6
  {1,1,1,0,0,0,0},  //7
  {1,1,1,1,1,1,1},  //8
  {1,1,1,1,0,1,1},  //9
  {1,1,1,0,1,1,1},  //A
  {0,0,1,1,1,1,1},  //b
  {1,0,0,1,1,1,0},  //C
  {0,1,1,1,1,0,1},  //d
  {1,0,0,1,1,1,1},  //E
  {1,0,0,0,1,1,1},  //F
  {0,0,0,0,0,0,0}   //blank
};

void displayPrint(char n) {
  for (int i = 0; i < 7; i++) {
    digitalWrite(seg[i], Chars[(int)n][i]);
  }
}

//---------------SD CARD SETUP---------------
SDMMCBlockDevice block_device;
mbed::FATFileSystem fs("fs");

static const char* ACC_FILE  = "/fs/TVac_Test/Acc_Data.dat";
static const char* GYRO_FILE = "/fs/TVac_Test/Gyro_Data.dat";
static const char* TEMP_FILE = "/fs/TVac_Test/Temp_Data.dat";
static const char* MAG_FILE  = "/fs/TVac_Test/Mag_Data.dat";

void removeOldDataFiles() {
  remove(ACC_FILE);
  remove(GYRO_FILE);
  remove(TEMP_FILE);
  remove(MAG_FILE);
}

void sd_write(const char* filename, const uint8_t* packet, size_t length) {
  FILE *mf = fopen(filename, "ab");
  if (!mf) {
    Serial.print("Failed to open file: ");
    Serial.println(filename);
    displayPrint(0x00);
    return;
  }
  fwrite(packet, 1, length, mf);
  fclose(mf);

  Serial.print("Data: ");
  Serial.write(packet, length);
  Serial.println();
}

void sd_init() {
  Serial.print("Do I get here?\n");
  int err = fs.mount(&block_device);
  Serial.print("First mounting tried\n");
  if (err) {
    err = fs.reformat(&block_device);}
  if (err) {
    Serial.println("Error formatting SDCARD");
    displayPrint(0x00);
    while (1) {}
  }
  mkdir("/fs/TVac_Test", 0777);
  removeOldDataFiles();
  Serial.println("SD init done, old data files removed");
}

//---------------I2C HELPER FUNCTIONS---------------
void bnoWriteReg(uint8_t reg, uint8_t val) {
  Wire1.beginTransmission(BNO_ADDRESS);
  Wire1.write(reg);
  Wire1.write(val);
  Wire1.endTransmission();
  delay(10);
}

uint8_t bnoReadReg(uint8_t reg) {
  Wire1.beginTransmission(BNO_ADDRESS);
  Wire1.write(reg);
  Wire1.endTransmission();
  Wire1.requestFrom(BNO_ADDRESS, (uint8_t)1);
  return Wire1.read();
}

void bnoReadRegs(uint8_t reg, uint8_t *buf, size_t len) {
  Wire1.beginTransmission(BNO_ADDRESS);
  Wire1.write(reg);
  Wire1.endTransmission();
  Wire1.requestFrom(BNO_ADDRESS, (uint8_t)len);
  for (size_t i = 0; i < len; i++) buf[i] = Wire1.read();
}

//---------------READ & CONVERT SENSORS---------------
// Raw accel in LSB; 1 LSB = 1 mg => g = raw * 0.001
void readRawAccel(int16_t &ax, int16_t &ay, int16_t &az) {
  uint8_t b[6];
  bnoReadRegs(REG_ACCEL_DATA, b, 6);
  ax = (int16_t)((b[1] << 8) | b[0]);
  ay = (int16_t)((b[3] << 8) | b[2]);
  az = (int16_t)((b[5] << 8) | b[4]);
}
float convertAccelG(int16_t raw) {
  return raw * 0.001f;
}

// Raw gyro in LSB; 16 LSB per °/s => dps = raw / 16
void readRawGyro(int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t b[6];
  bnoReadRegs(REG_GYRO_DATA, b, 6);
  gx = (int16_t)((b[1] << 8) | b[0]);
  gy = (int16_t)((b[3] << 8) | b[2]);
  gz = (int16_t)((b[5] << 8) | b[4]);
}
float convertGyroDPS(int16_t raw) {
  return raw / 16.0f;
}

// Raw mag in LSB; 16 LSB per µT => µT = raw / 16
void readRawMag(int16_t &mx, int16_t &my, int16_t &mz) {
  uint8_t b[6];
  bnoReadRegs(REG_MAG_DATA, b, 6);
  mx = (int16_t)((b[1] << 8) | b[0]);
  my = (int16_t)((b[3] << 8) | b[2]);
  mz = (int16_t)((b[5] << 8) | b[4]);
}
float convertMagUT(int16_t raw) {
  return raw / 16.0f;
}

// Ambient temp: 1 LSB = 1 °C (signed)
void readRawTempBNO(int8_t &tRaw) {
  tRaw = (int8_t)bnoReadReg(REG_TEMP);
}
float convertTempC_BNO(int8_t raw) {
  return (float)raw;
}

//---------------GLOBAL LOOP COUNTER---------------
static int g_roundCount = 0;

//---------------SETUP & LOOP---------------
void setup() {
  // 7-segment
  for (int i = 0; i < 8; i++) pinMode(seg[i], OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial) {}

  displayPrint(0x04);

  // SD card
  sd_init();

  // I2C1 on Portenta (defaults to D14/D15)
  Wire1.begin();
  Wire1.setClock(400000);

  // BNO055: reset & configure
  bnoWriteReg(REG_PWR_MODE, 0x00);                   // Normal power
  bnoWriteReg(REG_OPR_MODE, MODE_CONFIG);            // CONFIG mode
  delay(25);
  bnoWriteReg(REG_SYS_TRIGGER, 0x00);                // normal operation
  bnoWriteReg(REG_PAGE_ID, 0x00);                    // page 0
  bnoWriteReg(REG_UNIT_SEL, 0x00);                   // default units
  bnoWriteReg(REG_OPR_MODE, MODE_NDOF);              // NDOF fusion mode
  delay(20);

  Serial.println("BNO055 I²C init complete. Starting main loop...");
}

void loop() {
  g_roundCount++;
  Serial.println();
  Serial.print("Round ");
  Serial.print(g_roundCount);
  Serial.println(" of uploads:");

  // --- Read & convert ---
  int16_t gxRaw, gyRaw, gzRaw;
  readRawGyro(gxRaw, gyRaw, gzRaw);
  float gxDps = convertGyroDPS(gxRaw);
  float gyDps = convertGyroDPS(gyRaw);
  float gzDps = convertGyroDPS(gzRaw);

  int16_t axRaw, ayRaw, azRaw;
  readRawAccel(axRaw, ayRaw, azRaw);
  float axG = convertAccelG(axRaw);
  float ayG = convertAccelG(ayRaw);
  float azG = convertAccelG(azRaw);

  int16_t mxRaw, myRaw, mzRaw;
  readRawMag(mxRaw, myRaw, mzRaw);
  float mxUT = convertMagUT(mxRaw);
  float myUT = convertMagUT(myRaw);
  float mzUT = convertMagUT(mzRaw);

  int8_t tRawBNO;
  readRawTempBNO(tRawBNO);
  float tempC = convertTempC_BNO(tRawBNO);

  // --- Write GYRO ---
  {
    char outBuf[128];
    int len = sprintf(outBuf,
      "\nPacket #%d:\n"
      "Raw Gyro: GX:%d GY:%d GZ:%d\n"
      "Scaled Gyro: GX:%.2f GY:%.2f GZ:%.2f dps\n",
      g_roundCount,
      gxRaw, gyRaw, gzRaw,
      gxDps, gyDps, gzDps
    );
    sd_write(GYRO_FILE, (uint8_t*)outBuf, len);
  }

  // --- Write ACCEL ---
  {
    char outBuf[128];
    int len = sprintf(outBuf,
      "Packet #%d:\n"
      "Raw Acc: AX:%d AY:%d AZ:%d\n"
      "Scaled Acc: AX:%.3f AY:%.3f AZ:%.3f g\n",
      g_roundCount,
      axRaw, ayRaw, azRaw,
      axG, ayG, azG
    );
    sd_write(ACC_FILE, (uint8_t*)outBuf, len);
  }

  // --- Write MAG ---
  {
    char outBuf[128];
    int len = sprintf(outBuf,
      "Packet #%d:\n"
      "Raw Mag: MX:%d MY:%d MZ:%d\n"
      "Scaled Mag: MX:%.2f MY:%.2f MZ:%.2f uT\n",
      g_roundCount,
      mxRaw, myRaw, mzRaw,
      mxUT, myUT, mzUT
    );
    sd_write(MAG_FILE, (uint8_t*)outBuf, len);
  }

  // --- Write TEMP ---
  {
    char outBuf[128];
    int len = sprintf(outBuf,
      "Packet #%d:\n"
      "Raw Temp:%d\n"
      "Scaled Temp:%.2f C\n",
      g_roundCount,
      (int)tRawBNO,
      tempC
    );
    sd_write(TEMP_FILE, (uint8_t*)outBuf, len);
  }

  displayPrint(1);
  Serial.println();
  delay(10000);
}
