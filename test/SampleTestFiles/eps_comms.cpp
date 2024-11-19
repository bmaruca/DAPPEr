#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"
#include "mbed.h"
#include "Arduino_PortentaBreakout.h"
#include <Wire.h>

MbedI2C  myI2C2(PH_12,PH_11);

void setup() {
  Serial.begin(9600);
  myI2C2.begin(); // join i2c bus (address optional for writer)
}

void loop() {
  delay(1000);
  //Serial.println("begin trans");
  myI2C2.beginTransmission(0x2B); 
  myI2C2.write(0x10);
  myI2C2.write(0xE2);        // sends the given value
  myI2C2.write(0x80);
  myI2C2.endTransmission();    // stop transmitting
  delay(1);

myI2C2.requestFrom(0x2B, 2);  // Request numBytes from slave device

while (myI2C2.available() < 2) {
        // Wait until all requested bytes are available
        //Serial.println("waiting for bytes... ");
        delay(1);
    }
    for (int i = 0; i < 2; i++) {
      //Serial.println("Received Bytes");
        char receivedByte = myI2C2.read();  // Read each received byte
        Serial.print("Received byte ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(receivedByte, HEX);  // Print received data (optional)
        // Process received data as needed
    }
}