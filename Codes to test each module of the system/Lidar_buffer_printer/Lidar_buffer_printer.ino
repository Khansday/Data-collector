#include <Arduino.h>
#include "mbed.h"  // Include Mbed OS headers to apply mbed_app.json configurations


#define BUFFER_SIZE 1000 // Define the size of the buffer
byte raw_array[BUFFER_SIZE] = {0};  // Initialize the buffer
bool new_lidar_data_flag = false;

void setup() {
  Serial.begin(1000000);    // Start the serial port for logging
  Serial1.begin(230400);   // Start Serial1 to communicate with the LiDAR module

  delay(1000);
  Serial.println("Program Started");
  delay(1000);

}

void loop() {
  full_raw_buffer();   // Fill the buffer with data from LiDAR //this and next take 64.2 ms
  clearUARTBuffer();
  //print_raw_buffer();  // Print the received data
  
  if (new_lidar_data_flag) { parse_raw_buffer();} //next two up to 120 us
  else { Serial.println("No new data ");}
  //delay(10);
}

void full_raw_buffer() {
  bool timeout = false;
  Serial1.begin(230400);
  unsigned long timer = millis();
  size_t total_bytes_read = 0;

  while (total_bytes_read < BUFFER_SIZE ) {
    if (Serial1.available()) {
      int data = Serial1.read();
      if (data != -1) {
        raw_array[total_bytes_read++] = (byte)data;
      }
    }
    
    if (millis()-timer >= 1000){
      timeout = true;
      break;
    } 
  }
  if (!timeout){ new_lidar_data_flag = true;}
  else { Serial.println("Lidar timed out");  }
}

void parse_raw_buffer() {
  unsigned long timer = millis();
  new_lidar_data_flag = false;
  const int PACKET_SIZE = 46;
  const int NUM_POINTS = 12;
  int i = 0;
  while (i <= BUFFER_SIZE - PACKET_SIZE) {
    // Ensure we have enough data for a packet
    if (i + PACKET_SIZE > BUFFER_SIZE) {
      break;}
    if (raw_array[i] == 0x54 && raw_array[i + 1] == 0x2C) {
      // Found a packet
      // Process the packet
      int radar_speed = raw_array[i + 2] + (raw_array[i + 3] << 8); // little endian
      float start_angle = (raw_array[i + 4] + (raw_array[i + 5] << 8)) * 0.01;
      float end_angle = (raw_array[i + 42] + (raw_array[i + 43] << 8)) * 0.01;
      int timestamp = raw_array[i + 44] + (raw_array[i + 45] << 8);

      // Read data points
      int distances[NUM_POINTS];
      int signals[NUM_POINTS];

      for (int j = 0; j < NUM_POINTS; j++) {
        int idx = i + 6 + j * 3;
        int distance = raw_array[idx] + (raw_array[idx + 1] << 8);
        int signal = raw_array[idx + 2];
        distances[j] = distance;
        signals[j] = signal;
      }

      // Compute angle step
      float angle_diff = end_angle - start_angle;
      if (angle_diff < 0) {
        angle_diff += 360.0;
      }
      float step = angle_diff / (NUM_POINTS - 1);

      // Print the data points
      for (int k = 0; k < NUM_POINTS; k++) {
        float angle = start_angle + step * k;
        if (angle >= 360.0) {
          angle -= 360.0;
        }
        int distance = distances[k];
        int signal = signals[k];
        // Serial.print("Angle: ");
        // Serial.print(angle);
        // Serial.print("°, Distance: ");
        // Serial.print(distance);
        // Serial.print(" mm, Signal: ");
        // Serial.println(signal);

        // Log the data
        Serial.print(angle); // Angle rounded to nearest integer for display
        Serial.print("\t\t");
        Serial.print(distance);
        Serial.print("\t\t");
        Serial.println(signal);
      }

      // Move to the next packet
      i += PACKET_SIZE;
    } else {
      // Move to next byte
      i++;
    }

    if (millis()-timer >= 1000){
      break;
    } 
  }
}

void print_raw_buffer() {
  for (size_t i = 0; i < BUFFER_SIZE; i++) {
    Serial.print(raw_array[i], HEX);
    Serial.print(" ");
    // Ensure we don't access out-of-bounds indices
    if (i + 2 < BUFFER_SIZE && raw_array[i + 1] == 0x54 && raw_array[i + 2] == 0x2C) {
      Serial.println();
    }
  }
}

void clearUARTBuffer() {
  Serial1.end();
  while (Serial1.available() > 0) Serial1.read();
  Serial.flush();
  //Serial1.flush();
}
