#include "LidarProcessor.h"

// Lidar raw buffer (circular)
static const int BUFFER_SIZE = 10000;
static byte raw_array[BUFFER_SIZE] = {0};
static volatile size_t read_index = 0;
static volatile size_t write_index = 0;
static volatile size_t buffer_count = 0;
static bool new_lidar_data_flag = false;

// Exposed array for min distances
int min_distances[NUM_ANGLES];

/**
 * @brief Initialize min_distances with 8000 (some large value).
 */
void initDistanceArray() {
    for(int i=0; i<NUM_ANGLES; i++){
        min_distances[i] = 8000;
    }
}

/**
 * @brief Fill the ring buffer for up to timeLimit ms or until full.
 */
void fillBuffer(HardwareSerial &lidarSerial, unsigned long timeLimit) {
    unsigned long startTime = millis();
    while((millis() - startTime < timeLimit) && (buffer_count < BUFFER_SIZE)) {
        if(lidarSerial.available()) {
            int data = lidarSerial.read();
            if(data != -1) {
                if(buffer_count < BUFFER_SIZE) {
                    raw_array[write_index++] = (byte)data;
                    write_index %= BUFFER_SIZE;
                    buffer_count++;
                    new_lidar_data_flag = true;
                } else {
                    // Buffer is full
                    break;
                }
            }
        }
    }
}

/**
 * @brief Parse raw LiDAR data from the ring buffer for up to timeLimit ms,
 *        updating the min_distances array.
 */
void parseBuffer(unsigned long timeLimit) {
    if(!new_lidar_data_flag) return; // No new data to parse

    unsigned long startTime = millis();
    const int PACKET_SIZE = 46;
    const int NUM_POINTS  = 12;

    while((millis() - startTime < timeLimit) && (buffer_count >= PACKET_SIZE)) {
        // Check for packet start
        if(raw_array[read_index] == 0x54 && raw_array[(read_index+1)%BUFFER_SIZE] == 0x2C) {
            if(buffer_count >= PACKET_SIZE) {
                // Copy packet locally
                byte packet[PACKET_SIZE];
                for(int i=0; i<PACKET_SIZE; i++){
                    packet[i] = raw_array[(read_index + i) % BUFFER_SIZE];
                }

                float start_angle = (packet[4] + (packet[5]<<8))*0.01;
                float end_angle   = (packet[42] + (packet[43]<<8))*0.01;

                int start_angle_approx = ((int)(start_angle/10))*10;
                if(start_angle_approx >= 0 && start_angle_approx <= 180) {
                    int index = start_angle_approx/10;
                    int min_distance_in_packet = 8000;
                    
                    // read data points
                    for(int j=0; j<NUM_POINTS; j++){
                        int idx = 6 + j*3;
                        int distance = packet[idx] + (packet[idx+1]<<8);
                        if(distance != 0 && distance < min_distance_in_packet){
                            min_distance_in_packet = distance;
                        }
                    }
                    min_distances[index] = min_distance_in_packet;
                }

                // Advance read_index
                read_index = (read_index + PACKET_SIZE) % BUFFER_SIZE;
                buffer_count -= PACKET_SIZE;
            } else {
                // Not enough for a full packet
                break;
            }
        } else {
            // Skip 1 byte
            read_index = (read_index + 1) % BUFFER_SIZE;
            buffer_count--;
        }
    }
    // Done parsing
    new_lidar_data_flag = false;
}
