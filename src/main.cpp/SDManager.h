#ifndef SDMANAGER_H
#define SDMANAGER_H

#include <Arduino.h>
#include <SD.h>

// Declaration of functions related to SD card usage
bool initSDCard(int chipSelectPin);
File createNewFile(int &fileIndex);
void setColumnLegends(File &file, int numAngles);
void logData(File &file,
             unsigned long timestamp,
             float *receivedData,
             int numAngles,
             const int *minDistances,
             const String &utcTime,
             float latitude,
             float longitude,
             float speedKnots,
             float speedMph);

// Optionally, you could declare additional SD-related utilities here.

#endif
