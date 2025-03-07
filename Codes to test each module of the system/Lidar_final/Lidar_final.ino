/*
~~~~RETRIEVE PACKETS FROM LIDAR WITH TIMOUT AND FILLS A CIRCULAR BUFFER
~~~~NEW DATA IN THE BUFFER IS READ AND PROCESSED
~~~~ ALL ANGLES ARE ROUNDED TO LOWEST 10, PACKETS REGARDING TO THAT ANGLE, THE MIN DISTNACE IS UPDATED.
*/


#include <Arduino.h>
#include "mbed.h"  // Include Mbed OS headers to apply mbed_app.json configurations

#define BUFFER_SIZE 10000 // Define the size of the buffer
byte raw_array[BUFFER_SIZE] = {0};  // Initialize the buffer

// Circular buffer indices and counters
volatile size_t read_index = 0;
volatile size_t write_index = 0;
volatile size_t buffer_count = 0;

bool new_lidar_data_flag = false;

// Modifiable time limits (in milliseconds)
unsigned long buffer_fill_time_limit = 10; // Time limit for filling the buffer
unsigned long parsing_time_limit = 2;     // Time limit for parsing the buffer

const int NUM_ANGLES = 19; // Represents angles from 0 to 180 in steps of 10 degrees
int min_distances[NUM_ANGLES];

void setup() {
  Serial.begin(500000);    // Start the serial port for logging
  Serial1.begin(230400);   // Start Serial1 to communicate with the LiDAR module

  delay(1000);
  Serial.println("Program Started");
  delay(1000);

  init_distance_array();
}

void loop() {
  // Fill the buffer with data from LiDAR for a specified time or until full
  full_raw_buffer();

  // Parse the buffer at specified intervals with a time limit
  if (new_lidar_data_flag) {
    parse_raw_buffer_min_angle();
    new_lidar_data_flag = false; // Reset the flag after parsing
  } else {
    Serial.println("No new data");
  }
}

void full_raw_buffer() {
  unsigned long start_time = millis();

  while (millis() - start_time < buffer_fill_time_limit && buffer_count < BUFFER_SIZE) {
    if (Serial1.available()) {
      int data = Serial1.read();
      if (data != -1) {
        // Check if buffer is not full
        if (buffer_count < BUFFER_SIZE) {
          raw_array[write_index++] = (byte)data;
          write_index %= BUFFER_SIZE;
          buffer_count++;
          new_lidar_data_flag = true;
        } else {
          // Buffer is full
          Serial.println("Buffer is full, cannot write more data");
          break;
        }
      }
    }
  }

  if (millis() - start_time >= buffer_fill_time_limit) {
    Serial.println("Buffer fill time limit reached");
  }
}

void parse_raw_buffer_min_angle() {
  unsigned long start_time = millis();

  const int PACKET_SIZE = 46;
  const int NUM_POINTS = 12;

  while (millis() - start_time < parsing_time_limit && buffer_count >= PACKET_SIZE) {
    // Check for packet start
    if (raw_array[read_index] == 0x54 && raw_array[(read_index + 1) % BUFFER_SIZE] == 0x2C) {
      // Ensure we have enough data for a full packet
      if (buffer_count >= PACKET_SIZE) {
        // Read packet into a temporary array
        byte packet[PACKET_SIZE];
        for (int i = 0; i < PACKET_SIZE; i++) {
          packet[i] = raw_array[(read_index + i) % BUFFER_SIZE];
        }

        // Process the packet
        float start_angle = (packet[4] + (packet[5] << 8)) * 0.01;
        float end_angle = (packet[42] + (packet[43] << 8)) * 0.01;

        // Approximate start_angle to the lowest ten
        int start_angle_approx = ((int)(start_angle / 10)) * 10;

        // Check if start_angle_approx is between 0 and 180
        if (start_angle_approx >= 0 && start_angle_approx <= 180) {
          int index = start_angle_approx / 10;

          // Read data points and find the minimum distance in this packet
          int min_distance_in_packet = 65535;

          for (int j = 0; j < NUM_POINTS; j++) {
            int idx = 6 + j * 3;
            int distance = packet[idx] + (packet[idx + 1] << 8);
            // Ignore zero distances
            if (distance != 0 && distance < min_distance_in_packet) {
              min_distance_in_packet = distance;
            }
          }

          // Update the min_distances array 
          min_distances[index] = min_distance_in_packet;

          Serial.print(start_angle_approx);
          Serial.print("\t\t");
          Serial.print(min_distances[index]);
          Serial.print("\t\t");
          Serial.println("1");
          
        }

        // Move read_index forward by PACKET_SIZE
        read_index = (read_index + PACKET_SIZE) % BUFFER_SIZE;
        buffer_count -= PACKET_SIZE;
      } else {
        // Not enough data to read a full packet
        break;
      }
    } else {
      // Move read_index forward by 1
      read_index = (read_index + 1) % BUFFER_SIZE;
      buffer_count--;
    }
  }

  if (millis() - start_time >= parsing_time_limit) {
    Serial.println("Parsing time limit reached");
  }

  // After processing, print the updated min_distances array
  // for (int idx = 0; idx < NUM_ANGLES; idx++) {
  //   int angle = idx * 10;
  //   if (min_distances[idx] != 65535) {
  //     Serial.print(angle);
  //     Serial.print("\t\t");
  //     Serial.print(min_distances[idx]);
  //     Serial.print("\t\t");
  //     Serial.println("1");
  //   } else {
  //     Serial.print(angle);
  //     Serial.println("\t\tNo data");
  //   }
  // }
}

void init_distance_array(){
  // Initialize min_distances with a large value
  for (int idx = 0; idx < NUM_ANGLES; idx++) {
    min_distances[idx] = 65535;
  }
}
