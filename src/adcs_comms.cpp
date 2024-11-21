#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"
#include <Wire.h>
// === CHECKSUM FUNCTIONS ===

int generate_checksum(int buf[], int len){
  int sum = 0;
  while (--len){
    sum += buf[len];
  }
  return 0xff - sum + 1;
}

bool verify_checksum(int buf[], int len) {
  int sum = 0;
  while (--len){
    sum += buf[len];
  }
  return sum == 0;
}

//--------------------------------------

int totalBytes = 12; // Header + Body bytes, % 4 to find data count value

void setup(){  
    Serial.begin(115200);              
    Serial1.begin(115200);    // start serial for output
}
 
void loop()
{
  // HEADER (4 bytes)
  Serial1.write(0xC9); // 1st byte: read
  Serial1.write(0x01); // 2nd byte: index
  Serial1.write(0x03); // 3rd byte: data count
  Serial1.write((uint8_t)0x00); // 4th byte: map | error

  // BODY (totalBytes bytes)
  // (Read: returned from ADCS)
  // (Write: Values given by user)

  // CHECKSUM (1 byte)
  // First, populate buf array with header/body values
  int buf[totalBytes];
  int bytesRead = 0;
  while (bytesRead < totalBytes) {
    Serial.write("INFINITE POOP LOOP");
      if (Serial1.available()) {
          buf[bytesRead] = Serial1.read(); 
          bytesRead++;
      }
  }

  // Generate checksum
  Serial.write(generate_checksum(buf, totalBytes));
  //verify_checksum(buf, totalBytes);

  Serial.println("Waiting...");
  delay(1000);
}

