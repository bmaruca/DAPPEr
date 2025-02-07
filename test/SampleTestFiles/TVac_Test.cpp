#include "Arduino_PortentaBreakout.h"
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <serial.h>
#include <stdio.h>
#include "SDMMCBlockDevice.h"
#include "FATFileSystem.h"

//---------------PIN DEFINITIONS---------------
#define MPU_CS_PIN   PIN_SPI_SS         // MPU6500 chip select
#define MAG_CS_PIN   (PIN_SPI_SS + 1)   // MLX90395 chip select
#define MAG_DRDY_PIN A0                 // If used by MLX90395 (still unsure)

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
byte Chars[17][7]{
  { 1, 1, 1, 1, 1, 1, 0 },  //0
  { 0, 1, 1, 0, 0, 0, 0 },  //1
  { 1, 1, 0, 1, 1, 0, 1 },  //2
  { 1, 1, 1, 1, 0, 0, 1 },  //3
  { 0, 1, 1, 0, 0, 1, 1 },  //4
  { 1, 0, 1, 1, 0, 1, 1 },  //5
  { 1, 0, 1, 1, 1, 1, 1 },  //6
  { 1, 1, 1, 0, 0, 0, 0 },  //7
  { 1, 1, 1, 1, 1, 1, 1 },  //8
  { 1, 1, 1, 1, 0, 1, 1 },  //9
  { 1, 1, 1, 0, 1, 1, 1 },  //A
  { 0, 0, 1, 1, 1, 1, 1 },  //b
  { 1, 0, 0, 1, 1, 1, 0 },  //C
  { 0, 1, 1, 1, 1, 0, 1 },  //d
  { 1, 0, 0, 1, 1, 1, 1 },  //E
  { 1, 0, 0, 0, 1, 1, 1 },  //F
  { 0, 0, 0, 0, 0, 0, 0 }   //blank
};

void displayPrint(char n) {
  for (int i = 0; i < 7; i++) {
    digitalWrite(seg[i], Chars[(int)n][i]);
  }
}

//---------------SD CARD & FILE SETUP---------------
SDMMCBlockDevice block_device;
mbed::FATFileSystem fs("fs");

static const char* MAG_FILE  = "/fs/TVac_Test/Mag_Data.dat";
static const char* ACC_FILE  = "/fs/TVac_Test/Acc_Data.dat";
static const char* GYRO_FILE = "/fs/TVac_Test/Gyro_Data.dat";
static const char* TEMP_FILE = "/fs/TVac_Test/Temp_Data.dat";

// Remove the four known .dat files at startup
void removeOldDataFiles() {
  remove(MAG_FILE);
  remove(ACC_FILE);
  remove(GYRO_FILE);
  remove(TEMP_FILE);
}

// Write to the SD and print how many bytes
void sd_write(const char* filename, const uint8_t* packet, size_t length) {
  FILE *mf = fopen(filename, "ab");
  if(!mf) {
    Serial.print("Failed to open file: ");
    Serial.println(filename);
    return;
  }
  size_t bytesWritten = fwrite(packet, 1, length, mf);
  fclose(mf);

  Serial.print("Wrote ");
  Serial.print(bytesWritten);
  Serial.print(" bytes to ");
  Serial.println(filename);
}

// Initialize SD
void sd_init() {
  int err = fs.mount(&block_device);
  if (err) {
    // Reformat if mounting fails
    err = fs.reformat(&block_device);
  }
  if (err) {
    Serial.println("Error formatting SDCARD");
    while(1){}
  }

  // Ensure folder exists
  mkdir("/fs/TVac_Test", 0777);

  // Remove old data
  removeOldDataFiles();

  Serial.println("SD init done & old sensor data files removed.");
}

//---------------MPU6500 (Accel/Gyro/Temp) Over SPI + Scaling---------------
static const uint8_t REG_PWR_MGMT_1  = 0x6B;
static const uint8_t REG_USER_CTRL   = 0x6A;
static const uint8_t REG_FIFO_EN     = 0x23;
static const uint8_t REG_INT_PIN_CFG = 0x37;

void mpuWriteReg(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(MPU_CS_PIN, LOW);

  SPI.transfer(reg);
  SPI.transfer(val);

  digitalWrite(MPU_CS_PIN, HIGH);
  SPI.endTransaction();
  delay(10);
}

uint8_t mpuReadReg(uint8_t reg) {
  uint8_t cmd = reg | 0x80;
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(MPU_CS_PIN, LOW);
  SPI.transfer(cmd);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(MPU_CS_PIN, HIGH);
  SPI.endTransaction();
  return val;
}

void mpuReset() {
  mpuWriteReg(REG_PWR_MGMT_1, 0x80); // reset
  delay(100);
}

void configureMPU6500() {
  // Wake, set PLL
  mpuWriteReg(REG_PWR_MGMT_1, 0x01);
  delay(10);

  // Disable FIFO & I2C master
  mpuWriteReg(REG_USER_CTRL, 0x00);
  mpuWriteReg(REG_FIFO_EN, 0x00);

  // ±250 dps => 0x1B=0x00
  mpuWriteReg(0x1B, 0x00);
  // ±2g => 0x1C=0x00
  mpuWriteReg(0x1C, 0x00);

  // Possibly set 0x1A, 0x19 for DLPF & sample rate
  // Bypass if needed
  mpuWriteReg(REG_INT_PIN_CFG, 0x02);

  Serial.println("MPU6500 configured (±250 dps, ±2g).");
}

// Gyro
void readGyro(float &gx, float &gy, float &gz) {
  const float GYRO_SENS = 131.0f; // ±250 dps
  uint8_t reg = 0x43 | 0x80; // GYRO_XOUT_H
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(MPU_CS_PIN, LOW);
  SPI.transfer(reg);
  uint8_t xh = SPI.transfer(0x00);
  uint8_t xl = SPI.transfer(0x00);
  uint8_t yh = SPI.transfer(0x00);
  uint8_t yl = SPI.transfer(0x00);
  uint8_t zh = SPI.transfer(0x00);
  uint8_t zl = SPI.transfer(0x00);
  digitalWrite(MPU_CS_PIN, HIGH);
  SPI.endTransaction();

  int16_t rx = (int16_t)((xh << 8) | xl);
  int16_t ry = (int16_t)((yh << 8) | yl);
  int16_t rz = (int16_t)((zh << 8) | zl);

  gx = rx / GYRO_SENS;
  gy = ry / GYRO_SENS;
  gz = rz / GYRO_SENS;
}

// Accel
void readAccel(float &ax, float &ay, float &az) {
  const float ACC_SENS = 16384.0f; // ±2g
  uint8_t reg = 0x3B | 0x80;
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(MPU_CS_PIN, LOW);
  SPI.transfer(reg);
  uint8_t xh = SPI.transfer(0x00);
  uint8_t xl = SPI.transfer(0x00);
  uint8_t yh = SPI.transfer(0x00);
  uint8_t yl = SPI.transfer(0x00);
  uint8_t zh = SPI.transfer(0x00);
  uint8_t zl = SPI.transfer(0x00);
  digitalWrite(MPU_CS_PIN, HIGH);
  SPI.endTransaction();

  int16_t rx = (int16_t)((xh << 8) | xl);
  int16_t ry = (int16_t)((yh << 8) | yl);
  int16_t rz = (int16_t)((zh << 8) | zl);

  ax = rx / ACC_SENS;
  ay = ry / ACC_SENS;
  az = rz / ACC_SENS;
}

// Temp
void readMPUTemp(float &tempC) {
  uint8_t reg = 0x41 | 0x80;
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(MPU_CS_PIN, LOW);
  SPI.transfer(reg);
  uint8_t th = SPI.transfer(0x00);
  uint8_t tl = SPI.transfer(0x00);
  digitalWrite(MPU_CS_PIN, HIGH);
  SPI.endTransaction();

  int16_t raw = (int16_t)((th << 8) | tl);
  tempC = (raw / 333.87f) + 21.0f;
}

//---------------MLX90395 (Mag) Over SPI (Placeholder)---------------
uint16_t magT, magX, magY, magZ; 

void exitCmdMag(uint8_t cmd) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(MAG_CS_PIN, LOW);
  SPI.transfer(cmd);
  delayMicroseconds(11);
  SPI.transfer(0x00);
  digitalWrite(MAG_CS_PIN, HIGH);
  SPI.endTransaction();
  delay(5);
}

void burstCmdMag(uint8_t cmd) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(MAG_CS_PIN, LOW);
  SPI.transfer(cmd);
  delayMicroseconds(11);
  SPI.transfer(0x00);
  digitalWrite(MAG_CS_PIN, HIGH);
  SPI.endTransaction();
  delay(5);
}

// Possibly start measurement
uint8_t measureCmdMag(uint8_t cmd) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(MAG_CS_PIN, LOW);
  SPI.transfer(cmd);
  delayMicroseconds(11);
  uint8_t status = SPI.transfer(0x00);
  digitalWrite(MAG_CS_PIN, HIGH);
  SPI.endTransaction();
  delay(5);
  return status;
}

// read raw T,X,Y,Z
uint8_t readCmdMag(uint8_t cmd) {
  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
  digitalWrite(MAG_CS_PIN, LOW);
  SPI.transfer(cmd);
  delayMicroseconds(11);
  uint8_t status = SPI.transfer(0x00);

  magT = SPI.transfer16(0x00);
  magX = SPI.transfer16(0x00);
  magY = SPI.transfer16(0x00);
  magZ = SPI.transfer16(0x00);
  uint8_t crcC = SPI.transfer(0x00); // ignoring CRC
  digitalWrite(MAG_CS_PIN, HIGH);
  SPI.endTransaction();
  return status;
}

// Ccale placeholders
void scaleMagData(float &tVal, float &xVal, float &yVal, float &zVal) {
  // Real factor unknown, I must be blind
  tVal = magT * 0.1f;
  xVal = magX * 0.01f;
  yVal = magY * 0.01f;
  zVal = magZ * 0.01f;
}

//---------------GLOBAL LOOP COUNTER---------------
static int g_roundCount = 0;

//---------------SETUP & LOOP---------------
void setup() {
  // 7-seg pins
  for (int i = 0; i < 8; i++) {
    pinMode(seg[i], OUTPUT);
  }
  pinMode(MPU_CS_PIN, OUTPUT);
  digitalWrite(MPU_CS_PIN, HIGH);

  pinMode(MAG_CS_PIN, OUTPUT);
  digitalWrite(MAG_CS_PIN, HIGH);

  pinMode(MAG_DRDY_PIN, INPUT); // if used

  Serial.begin(115200);
  while(!Serial){}

  // Init SD (wiping old data files)
  sd_init();

  // Start SPI
  SPI.begin();

  // MPU6500
  mpuReset();
  configureMPU6500();
  uint8_t who = mpuReadReg(0x75);
  Serial.print("MPU6500 WHO_AM_I=0x");
  Serial.println(who, HEX);

  // MLX90395
  exitCmdMag(0x80);
  delayMicroseconds(1000);
  exitCmdMag(0xF0);
  burstCmdMag(0x1F);
  Serial.println("MLX90395 init done.");

  Serial.println("Setup complete. Beginning data logging...");
}

void loop() {
  // Increase round
  g_roundCount++;

  // Print "Round N" in Serial
  Serial.println();
  Serial.print("Round ");
  Serial.print(g_roundCount);
  Serial.println(" of uploads:");

  // MPU6500 reads 
  float gx, gy, gz;
  readGyro(gx, gy, gz);

  float ax, ay, az;
  readAccel(ax, ay, az);

  float tC;
  readMPUTemp(tC);

  // GYRO
  {
    // Combine "Packet #N" and data into one string
    char outBuf[128];
    int len = sprintf(outBuf,
      "Packet #%d:\n"
      "GX:%.2f GY:%.2f GZ:%.2f dps\n",
      g_roundCount, gx, gy, gz);
    sd_write(GYRO_FILE, (uint8_t*)outBuf, len);
  }

  // ACCEL
  {
    char outBuf[128];
    int len = sprintf(outBuf,
      "Packet #%d:\n"
      "AX:%.3f AY:%.3f AZ:%.3f g\n",
      g_roundCount, ax, ay, az);
    sd_write(ACC_FILE, (uint8_t*)outBuf, len);
  }

  // TEMP
  {
    char outBuf[128];
    int len = sprintf(outBuf,
      "Packet #%d:\n"
      "TMP:%.2f C\n",
      g_roundCount, tC);
    sd_write(TEMP_FILE, (uint8_t*)outBuf, len);
  }

  // MLX90395 reads
  measureCmdMag(0x3E); // if 0x3E is correct for our sensor
  delay(20);

  // optional: while(digitalRead(MAG_DRDY_PIN) != HIGH){(wait)}

  uint8_t magStatus = readCmdMag(0x4F);
  float tVal, xVal, yVal, zVal;
  scaleMagData(tVal, xVal, yVal, zVal);

  {
    char outBuf[128];
    int len = sprintf(outBuf,
      "Packet #%d:\n"
      "T:%.2f X:%.2f Y:%.2f Z:%.2f\n",
      g_roundCount, tVal, xVal, yVal, zVal);
    sd_write(MAG_FILE, (uint8_t*)outBuf, len);
  }

  // Visual confirmation
  displayPrint(1);

  // Blank line in serial
  Serial.println();

  // wait 10 seconds
  delay(10000);
}