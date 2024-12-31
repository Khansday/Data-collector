#ifndef LIDARPROCESSOR_H
#define LIDARPROCESSOR_H

#include <Arduino.h>
#include <HardwareSerial.h> // or <UART.h> if needed

/**
 * @brief Number of angles from 0..180 in steps of 10 => 19 total
 */
#define NUM_ANGLES 19

// External array for minimum distances (so main can read them)
extern int min_distances[NUM_ANGLES];

/**
 * @brief Initialize the LiDAR distance array with some default large value.
 */
void initDistanceArray();

/**
 * @brief Reads from the Serial1 (or whichever) and fills a ring buffer.
 */
void fillBuffer(HardwareSerial &lidarSerial, unsigned long timeLimit);

/**
 * @brief Parses the ring buffer to update the min_distances array.
 */
void parseBuffer(unsigned long timeLimit);

#endif
