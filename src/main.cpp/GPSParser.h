#ifndef GPSPARSER_H
#define GPSPARSER_H

#include <Arduino.h>
#include <HardwareSerial.h>  // Or <UART.h> depending on your environment

// Expose these so main can see them (or store them in a struct)
extern float latitude;
extern float longitude;
extern float speed_knots;
extern float speed_mph;
extern String utcTime;

/**
 * @brief Initialize the GPS (optional).
 *        For example, set the starting baud or do other config.
 */
void initGPS(UART &uartGPS, unsigned long initialBaud);

/**
 * @brief Sets the GPS module’s baud rate using a custom command
 */
void setGPSBaudRate(UART &uartGPS, uint32_t newBaudRate);

/**
 * @brief Reads and parses data from the GPS module.
 *        Internally calls parseGGA, parseRMC, etc.
 */
void GPSParser(UART &uartGPS, unsigned long timeLimit);

/**
 * @brief Helper to parse GGA sentences
 */
void parseGGA(const String &sentence);

/**
 * @brief Helper to parse RMC sentences
 */
void parseRMC(const String &sentence);

#endif
