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

// SSD segment pins
breakoutPin A = PWM9;
breakoutPin B = PWM8;
breakoutPin C = PWM7;
breakoutPin D = PWM6;
breakoutPin E = PWM5;
breakoutPin Z = PWM4;
breakoutPin G = PWM3;
breakoutPin DP = PWM2;

int seg[]{ A, B, C, D, E, Z, G, DP };
byte Chars[17][7] = {
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
  { 1, 1, 1, 0, 1, 1, 1 },  //A/10
  { 0, 0, 1, 1, 1, 1, 1 },  //b/11
  { 1, 0, 0, 1, 1, 1, 0 },  //C/12
  { 0, 1, 1, 1, 1, 0, 1 },  //d/13
  { 1, 0, 0, 1, 1, 1, 1 },  //E/14
  { 1, 0, 0, 0, 1, 1, 1 },  //F/15
  { 0, 0, 0, 0, 0, 0, 0 }   //blank
};

void displayPrint(char n) {
  for (char i = 0; i < 7; i++) {
    digitalWrite(seg[i], Chars[n][i]);
  }
}

void setup() {
  Serial.begin(115200);     // Debug
  Serial3.begin(115200);    // UART3 for RE comms

  // SSD setup
  for(int i = 0; i < 8; i++) pinMode(seg[i], OUTPUT);

  displayPrint(0);          // Display zero on startup
}

void loop() {
  static bool started = false;
  static unsigned long startMillis = 0;

  // Wait for "START" command from RE
  if (!started) {
    if (Serial3.available()) {
      String cmd = Serial3.readStringUntil('\n');
      Serial.print("FC received: "); Serial.println(cmd); // For debugging
      cmd.trim();
      if (cmd.equalsIgnoreCase("START")) {
        started = true;
        startMillis = millis();
      }
    }
    return; // Stay here until started
  } 

  // Cycle 0-F, forever
  while (true) {
    for (int i = 0; i < 16; i++) {
      displayPrint(i);

      // Calculate elapsed time since start of cycling (not power up)
      unsigned long elapsed = millis() - startMillis;

      // Send digit + elapsed time to RE
      char buf[40];
      sprintf(buf, "SSD: %X | t=%lus\n", i, elapsed / 1000);
      Serial3.print(buf);

      delay(1000); // 1 sec between digits
    }
  }
}
