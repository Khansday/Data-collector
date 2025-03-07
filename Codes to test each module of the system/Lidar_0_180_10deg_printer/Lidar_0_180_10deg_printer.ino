#include <Arduino.h>
#include "mbed.h"  // Include Mbed OS headers to apply mbed_app.json configurations


#define BUFFER_SIZE 3000 // Define the size of the buffer
byte raw_array[BUFFER_SIZE] = {0};  // Initialize the buffer
bool new_lidar_data_flag = false;

unsigned long long int timer;
const int NUM_ANGLES = 19; // Represents angles from 0 to 170 in steps of 10 degrees

int min_distances[NUM_ANGLES];

void setup() {
  Serial.begin(500000);    // Start the serial port for logging
  Serial1.begin(230400);   // Start Serial1 to communicate with the LiDAR module

  delay(1000);
  Serial.println("Program Started");
  delay(1000);

}

void loop() {

  full_raw_buffer();   // Fill the buffer with data from LiDAR 
  //clearUARTBuffer();

  //print_raw_buffer();  // Print the received data
  if (new_lidar_data_flag) { 
    //parse_raw_buffer();
    parse_raw_buffer_min_angle();
  }
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
      break;
    }
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

      // Process the data points
      for (int k = 0; k < NUM_POINTS; k++) {
        float angle = start_angle + step * k;
        if (angle >= 360.0) {
          angle -= 360.0;
        }
        int approx_angle = ((int)(angle / 10)) * 10; // Approximate to the lowest multiple of 10

        if (approx_angle >= 0 && approx_angle <= 180) {
          int distance = distances[k];
          int signal = signals[k];
          if (distance != 0) { // Ensure the distance is not zero
            // Log the data
            Serial.print(approx_angle);
            Serial.print("\t\t");
            Serial.print(distance);
            Serial.print("\t\t");
            Serial.println(signal);
          }
        }
      }

      // Move to the next packet
      i += PACKET_SIZE;
    } else {
      // Move to next byte
      i++;
    }

    if (millis() - timer >= 1000) {
      break;
    }
  }
}

void parse_raw_buffer_min_angle() {
  unsigned long timer = millis();
  new_lidar_data_flag = false;
  const int PACKET_SIZE = 46;
  const int NUM_POINTS = 12;

  // Initialize min_distances with a large value
  for (int idx = 0; idx < NUM_ANGLES; idx++) {
    min_distances[idx] = 65535;
  }

  int i = 0;
  while (i <= BUFFER_SIZE - PACKET_SIZE) {
    // Ensure we have enough data for a packet
    if (i + PACKET_SIZE > BUFFER_SIZE) {
      break;
    }
    if (raw_array[i] == 0x54 && raw_array[i + 1] == 0x2C) {
      // Found a packet
      // Process the packet
      //int radar_speed = raw_array[i + 2] + (raw_array[i + 3] << 8); // little endian
      float start_angle = (raw_array[i + 4] + (raw_array[i + 5] << 8)) * 0.01;
      float end_angle = (raw_array[i + 42] + (raw_array[i + 43] << 8)) * 0.01;
      //int timestamp = raw_array[i + 44] + (raw_array[i + 45] << 8);

      // Approximate start_angle to the lowest ten
      int start_angle_approx = ((int)(start_angle / 10)) * 10;

      // Check if start_angle_approx is between 0 and 170
      if (start_angle_approx >= 0 && start_angle_approx <= 180) {
        int index = start_angle_approx / 10;

        // Read data points and find the minimum distance in this packet
        int min_distance_in_packet = 65535;

        for (int j = 0; j < NUM_POINTS; j++) {
          int idx = i + 6 + j * 3;
          int distance = raw_array[idx] + (raw_array[idx + 1] << 8);
          // Ignore zero distances
          if (distance != 0 && distance < min_distance_in_packet) {
            min_distance_in_packet = distance;
          }
        }

        // Update the min_distances array if a smaller distance is found
        if (min_distance_in_packet < min_distances[index]) {
          min_distances[index] = min_distance_in_packet;
        }
      }

      // Move to the next packet
      i += PACKET_SIZE;
    } else {
      // Move to next byte
      i++;
    }

    if (millis() - timer >= 1000) {
      break;
    }
  }

  // After processing all data, print the min_distances array
  for (int idx = 0; idx < NUM_ANGLES; idx++) {
    int angle = idx * 10;
    if (min_distances[idx] != 65535) {
       Serial.print(angle);
      Serial.print("\t\t");
      Serial.print(min_distances[idx]);
      Serial.print("\t\t");
      Serial.println("1");
    } else {
      Serial.println("No data");
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
