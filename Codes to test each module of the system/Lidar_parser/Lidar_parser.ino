#include <Arduino.h>

// Initialize Serial ports
void setup() {
  Serial.begin(250000);      // Start the hardware serial port for logging
  Serial1.begin(230400);     // Start the software serial port to communicate with the LiDAR
}

// Main loop to handle serial data
void loop() {
  static byte dataPacket[44];
  static int byteCount = 0;

  while (Serial1.available()) {
    byte incomingByte = Serial1.read();
    if (byteCount == 0 && incomingByte != 0x54) {
      // First byte must be 0x54, skip any bytes until this is found
      continue;
    }

    dataPacket[byteCount++] = incomingByte;

    // Check if we have a complete packet
    if (byteCount == 44) {
      // Verify packet start and descriptor byte
      if (dataPacket[0] == 0x54 && dataPacket[1] == 0x2C) {
        //processPacket(dataPacket);
        printRawpacket(dataPacket);

      }
      // Reset byte count to start new packet capture
      byteCount = 0;
    }
  }
  delayMicroseconds(10); // Small delay to stabilize loop
}

// Function to process a complete data packet
void processPacket(byte* packet) {
  int radarSpeed = packet[2] + (packet[3] << 8);
  float startAngle = (packet[4] + (packet[5] << 8)) * 0.01;
  float endAngle = (packet[40] + (packet[41] << 8)) * 0.01;
  int timestamp = packet[42] + (packet[43] << 8);

  float step = (endAngle - startAngle) / 11; // Correctly compute for 12 data points
  for (int i = 6, j = 0; i < 40; i += 3, j++) {
    int distance = packet[i] + (packet[i+1] << 8);
    byte signalStrength = packet[i+2];
    float angle = startAngle + step * j;
    // Log the data
    Serial.print(int(angle)); // Angle rounded to nearest integer for display
    Serial.print("\t\t");
    Serial.print(distance);
    Serial.print("\t\t");
    Serial.println(signalStrength);
  }
}

// Function to print the raw data of a packet
void printRawpacket(const byte* packet) {
  //Serial.println("Raw Packet Data:");
  for (int i = 0; i < 44; i++) {
    Serial.print(packet[i], HEX); // Print each byte in hexadecimal
    Serial.print(" ");
  }
  Serial.println(); // New line after printing the whole packet
}
