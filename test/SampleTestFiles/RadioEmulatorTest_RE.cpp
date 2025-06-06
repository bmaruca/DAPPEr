/*This code DOES NOT work in this workspace as it is setup for the Portenta. Copy this code into the Arduino IDE and upload from there

#define FC_BAUD 115200 // Match Portenta's Serial3 baud

void setup() {
  Serial.begin(115200);    // USB Serial to your PC
  
  // Wait for Serial to initialize
  while (!Serial) {
    ; // Do nothing until Serial is ready
  }

  // Specify RX, TX pins for Nano ESP32!
  Serial1.begin(FC_BAUD, SERIAL_8N1, 43, 44);  // RX=44 (D0), TX=43 (D1)

  delay(100);              // Let everything power up
  
  // Send START command to FC
  Serial1.println("START");
  Serial.println("Sent 'START' to Flight Computer.");
}

void loop() {
  // If data comes from FC over Serial1, relay to PC Serial Monitor
  if (Serial1.available()) {
    String line = Serial1.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      Serial.print("FC: ");
      Serial.println(line);  // e.g. "A,12"
    }
  }
}

*/