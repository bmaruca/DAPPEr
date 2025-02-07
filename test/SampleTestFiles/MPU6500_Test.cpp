#include "Arduino_PortentaBreakout.h"
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <serial.h>
#include "SDMMCBlockDevice.h"
#include "FATFileSystem.h"
#include <stdio.h>

//---------------PIN & REGISTER DEFINITIONS---------------
#define LAST_ARDUINO_PIN_NUMBER LEDB + 1

static uint8_t constexpr REGISTER_PWR_MGMT_1      = 0x6B;
static uint8_t constexpr REGISTER_VALUE_RESET     = 0x80;
static uint8_t constexpr REGISTER_INT_PIN_CFG     = 0x37;
static uint8_t constexpr REGISTER_VALUE_BYPASS_EN = 0x02;

// Accelerometer, Gyro, Temp start registers
static uint8_t constexpr ACCEL_XOUT_H = 0x3B;
static uint8_t constexpr GYRO_XOUT_H  = 0x43;
static uint8_t constexpr TEMP_OUT_H   = 0x41;

// SPI pins
static const uint8_t MOSI_PIN = PC_3;
static const uint8_t MISO_PIN = PC_2;
static const uint8_t SCLK_PIN = PI_1;
static const uint8_t CS_PIN   = LAST_ARDUINO_PIN_NUMBER + PD_4;
static const uint8_t INT_PIN  = PG_3;

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
byte Chars[17][7] {
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

//---------------SD CARD SETUP---------------
SDMMCBlockDevice block_device;
mbed::FATFileSystem fs("fs");

static const char* ACC_FILE  = "/fs/TVac_Test/Acc_Data.dat";
static const char* GYRO_FILE = "/fs/TVac_Test/Gyro_Data.dat";
static const char* TEMP_FILE = "/fs/TVac_Test/Temp_Data.dat";

// Remove the 3 known data files each time to start fresh
void removeOldDataFiles() {
  remove(ACC_FILE);
  remove(GYRO_FILE);
  remove(TEMP_FILE);
}

// Write to the SD and print how many bytes
void sd_write(const char* filename, const uint8_t* packet, size_t length) {
  FILE *mf = fopen(filename, "ab");
  if (!mf) {
    Serial.print("Failed to open file: ");
    Serial.println(filename);
    displayPrint(0x00);
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
    Serial.println("Error formatting SDCARD ");
    displayPrint(0x00);
    while (1) {}
  }
  // Ensure folder exists
  mkdir("/fs/TVac_Test", 0777);

  // Remove old data
  removeOldDataFiles();

  Serial.println("SD init done, old data files removed");
}

//---------------SPI REGISTER R/W FUNCTIONS---------------
// Write a single register
void gyroWriteReg(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);

  SPI.transfer(reg);
  SPI.transfer(val);

  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  delay(10);
}

// Read WHO_AM_I
uint8_t gyroWHOAMI(uint8_t reg) {
  reg |= 0x80; // Set read bit
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);

  SPI.transfer(reg);
  uint8_t status = SPI.transfer(0x00);

  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();

  Serial.print("WHO_AM_I register read: 0x");
  Serial.println(status, HEX);
  return status;
}

// Reset device
void gyroReset(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);

  SPI.transfer(reg);
  SPI.transfer(val);

  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  delay(10);
}

//---------------READING THE RAW SENSOR DATA, THEN SCALING IT---------------
// Read raw gyro data
void readRawGyro(int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t reg = GYRO_XOUT_H | 0x80; // read bit
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);

  SPI.transfer(reg);
  uint8_t xh = SPI.transfer(0x00);
  uint8_t xl = SPI.transfer(0x00);
  uint8_t yh = SPI.transfer(0x00);
  uint8_t yl = SPI.transfer(0x00);
  uint8_t zh = SPI.transfer(0x00);
  uint8_t zl = SPI.transfer(0x00);

  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();

  gx = (int16_t)((xh << 8) | xl);
  gy = (int16_t)((yh << 8) | yl);
  gz = (int16_t)((zh << 8) | zl);
}

// Read raw accel data
void readRawAccel(int16_t &ax, int16_t &ay, int16_t &az) {
  uint8_t reg = ACCEL_XOUT_H | 0x80;
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);

  SPI.transfer(reg);
  uint8_t xh = SPI.transfer(0x00);
  uint8_t xl = SPI.transfer(0x00);
  uint8_t yh = SPI.transfer(0x00);
  uint8_t yl = SPI.transfer(0x00);
  uint8_t zh = SPI.transfer(0x00);
  uint8_t zl = SPI.transfer(0x00);

  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();

  ax = (int16_t)((xh << 8) | xl);
  ay = (int16_t)((yh << 8) | yl);
  az = (int16_t)((zh << 8) | zl);
}

// Read raw temp
void readRawTemp(int16_t &rawTemp) {
  uint8_t reg = TEMP_OUT_H | 0x80;
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);

  SPI.transfer(reg);
  uint8_t th = SPI.transfer(0x00);
  uint8_t tl = SPI.transfer(0x00);

  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();

  rawTemp = (int16_t)((th << 8) | tl);
}

//---------------SCALING FACTORS (assuming ±250 dps, ±2g range)---------------
float convertGyroDPS(int16_t raw) {
  // ±250 dps => 131 LSB/(°/s)
  const float GYRO_SENS = 131.0f;
  return raw / GYRO_SENS;
}

float convertAccelG(int16_t raw) {
  // ±2g => 16384 LSB/g
  const float ACC_SENS = 16384.0f;
  return raw / ACC_SENS;
}

float convertTempC(int16_t raw) {
  // (raw / 333.87) + 21
  return (raw / 333.87f) + 21.0f;
}


//---------------GLOBAL LOOP COUNTER FOR "Round #N"---------------
static int g_roundCount = 0;

//---------------SETUP & LOOP---------------
void setup() {
  // 7-seg pins
  for (int i = 0; i < 8; i++) {
    pinMode(seg[i], OUTPUT);
  }

  pinMode(INT_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  while (!Serial) {}

  // Init SD (wiping old data files)
  sd_init();

  // Setup SPI
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin();

  // Minimal initialization
  gyroReset(REGISTER_PWR_MGMT_1, REGISTER_VALUE_RESET);
  delay(10);

  // Bypass mode if needed
  gyroWriteReg(REGISTER_INT_PIN_CFG, REGISTER_VALUE_BYPASS_EN);
  delay(10);

  // Check WHO_AM_I
  gyroWHOAMI(0x75);

  Serial.println("MPU6500 SPI init complete. Starting main loop...");
}

void loop() {
  // Increase round
  g_roundCount++;

  // Print "Round N" in Serial
  Serial.println();
  Serial.print("Round ");
  Serial.print(g_roundCount);
  Serial.println(" of uploads:");

  // Read raw gyro
  int16_t gxRaw, gyRaw, gzRaw;
  readRawGyro(gxRaw, gyRaw, gzRaw);

  // Read raw accel
  int16_t axRaw, ayRaw, azRaw;
  readRawAccel(axRaw, ayRaw, azRaw);

  // Read raw temp
  int16_t tRaw;
  readRawTemp(tRaw);

  // Convert them to scaled data
  float gxDps = convertGyroDPS(gxRaw);
  float gyDps = convertGyroDPS(gyRaw);
  float gzDps = convertGyroDPS(gzRaw);

  float axG = convertAccelG(axRaw);
  float ayG = convertAccelG(ayRaw);
  float azG = convertAccelG(azRaw);

  float tempC = convertTempC(tRaw);

  // GYRO
  {
    char outBuf[128];
    int len = sprintf(outBuf,
      "Packet #%d:\n"
      "Raw Gyro: GX:%d GY:%d GZ:%d\n"
      "Scaled Gyro: GX:%.2f GY:%.2f GZ:%.2f dps\n",
      g_roundCount,
      gxRaw, gyRaw, gzRaw,
      gxDps, gyDps, gzDps
    );
    sd_write(GYRO_FILE, (uint8_t*)outBuf, len);
  }

  // ACCEL
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

  // TEMP
  {
    char outBuf[128];
    int len = sprintf(outBuf,
      "Packet #%d:\n"
      "Raw Temp:%d\n"
      "Scaled Temp:%.2f C\n",
      g_roundCount,
      tRaw,
      tempC
    );
    sd_write(TEMP_FILE, (uint8_t*)outBuf, len);
  }

  // Visual confirmation
  displayPrint(1);

  // Blank line in serial
  Serial.println();

  // Wait 10 seconds
  delay(10000);
}
