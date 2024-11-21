#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"
#include "mbed.h"
#include "Arduino_PortentaBreakout.h"
#include <Wire.h>

// === COMMUNICATION ===
BreakoutCarrierClass Breakout;
UART myUART = Breakout.UART1;

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

bool myIncoming = false;
int myLastUART = -1;
int totalBytes = 12; // Header + Body bytes, % 4 to find data count value

void setup(){  
    pinMode(LEDB, OUTPUT);   // LEDB = blue, LEDG or LED_BUILTIN = green, LEDR = red
 
    Serial.begin(115200);    // start serial for output
    myUART.begin(115200); // baud rate
}
 
void loop()
{
  // HEADER (4 bytes)
  myUART.write(0xC9); // 1st byte: read
  myUART.write(0x01); // 2nd byte: index
  myUART.write(0x03); // 3rd byte: data count
  myUART.write((uint8_t)0x00); // 4th byte: map | error

  // BODY (totalBytes bytes)
  // (Read: returned from ADCS)
  // (Write: Values given by user)

  // CHECKSUM (1 byte)
  // First, populate buf array with header/body values
  int buf[totalBytes];
  int bytesRead = 0;
  while (bytesRead < totalBytes) {
      if (myUART.available()) {
          buf[bytesRead] = myUART.read(); 
          bytesRead++;
      }
  }

  printf("%d", generate_checksum(buf, totalBytes));        // print the character

  // Generate checksum
  myUART.write(generate_checksum(buf, totalBytes));


  //verify_checksum(buf, totalBytes);

  delay(1);
 
 // DEBUGGING
 /*
 
  while(myUART.available()){     // slave may send less than requested
    //Serial.print("bytes available");        // print the character
    myIncoming = true;
    char myChar = myUART.read(); // receive a byte as character (subtracts byte)
    Serial.print(myChar);        // print the character
    myLastUART = 2;
  }
 
  if (myIncoming) {  
      Serial.println();        // just nicely finishes the line.
      myIncoming = false;
  }
 
  if (myLastUART >=0) {
     Serial.println("Last Serial message was from UART1:" + String(myLastUART));
     myLastUART = -1;
  }
  */
 
  Serial.println("Waiting...");
  delay(1000);
  digitalWrite(LEDB, !digitalRead(LEDB));   //switch back and forth on, off
}

