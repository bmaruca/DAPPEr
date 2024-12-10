// Matthew Ward, Christian Keleher, Gia Scozzaro
// This code is intended to communicate with the TensorTech ADCS, using the most basic command: get SNID. 
// This code is written to be used with the Arduino Due, using RX1 and TX1 for UART Communications


#include <Arduino.h>

#define TOTAL_BYTES 17  //Adjust based on amount of bytes expected upon return

//Function to sum all the bytes sent in the header
uint8_t generate_checksum(uint8_t buf[], int len) {
    int sum = 0;
    for (int i = 0; i < len - 1; i++) { // Exclude checksum byte
        sum += buf[i];
    }
    return (uint8_t)(0xFF - (sum & 0xFF) + 1);
}
//Function to sum all of the bytes returned from ADCS 
bool verify_checksum(uint8_t buf[], int len) {
    int sum = 0;
    for (int i = 0; i < len; i++) {
        sum += buf[i];
    }
    return (sum & 0xFF) == 0;
}

void setup() {   
    Serial.begin(115200);    // Start serial for USB debugging
    Serial1.begin(115200, SERIAL_8N1);   // Start UART for ADCS
    //IMPORTANT: You must configure monitor_speed = 115200 in platformio.ini to be able to read in the serial monitor
}

void loop() {
    // HEADER (4 bytes)
    uint8_t header[] = {0xC9, 0x01, 0x03, 0x00};
    uint8_t checksum = generate_checksum(header, 4);
    //Serial.print(checksum,HEX);
    Serial1.write(header, 4);  // Send header
    Serial1.write(checksum);   // Send checksum


    // Read response
    uint8_t buf[TOTAL_BYTES];
    int bytesRead = 0;
    unsigned long startTime = millis();
    
    while (bytesRead < TOTAL_BYTES && (millis() - startTime) < 1000) { // 1-second timeout
        if (Serial1.available()) {
            buf[bytesRead++] = Serial1.read();
        }
    }

    // Check if we got all bytes
    if (bytesRead < TOTAL_BYTES) {
        Serial.println("Error: Incomplete response");
        return;
    }

    // Print received bytes
    for (int i = 0; i < TOTAL_BYTES; i++) {
        Serial.print("Byte ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(char (buf[i]));
    }

    // Verify checksum
    if (verify_checksum(buf, TOTAL_BYTES)) {
        Serial.println("Checksum valid!");
    } else {
        Serial.println("Checksum invalid!");
    }

    delay(1000); // Delay before sending the next command
}
