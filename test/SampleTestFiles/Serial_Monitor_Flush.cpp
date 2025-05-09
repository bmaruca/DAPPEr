#include "Arduino_PortentaBreakout.h"
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "SDMMCBlockDevice.h"
#include "FATFileSystem.h"
#include <stdio.h>

#define LAST_ARDUINO_PIN_NUMBER LEDB + 1

void setup() {
  Serial.begin(115200);
}

void loop() {
    Serial.print("Hello\n");
    delay(100);
}
