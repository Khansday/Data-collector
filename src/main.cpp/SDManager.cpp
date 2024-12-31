#include "SDManager.h"

/**
 * @brief Initialize the SD card.
 * @return True if initialization was successful, otherwise false.
 */
bool initSDCard(int chipSelectPin) {
    SD.begin(chipSelectPin);
    delay(800);
    // Attempt to begin
    if(SD.begin(chipSelectPin)) {
        Serial.println("SD card initialization done.");
        return true;
    } else {
        Serial.println("SD Initialization failed!");
        return false;
    }
}

/**
 * @brief Creates a new CSV file with an incremental index.
 * @param fileIndex A reference to the current file index.
 * @return A File object opened for writing (FILE_WRITE).
 */
File createNewFile(int &fileIndex) {
    File dataFile;
    while(true) {
        String fileName = "Data" + String(fileIndex) + ".csv";
        // If file does not exist, create and open
        if (!SD.exists(fileName)) {
            dataFile = SD.open(fileName, FILE_WRITE);
            if (dataFile) {
                Serial.print("Created new file: ");
                Serial.println(fileName);
                fileIndex++;
                break;
            } else {
                Serial.println("Error creating file!");
            }
        } else {
            fileIndex++;  // If it exists, try next
        }
    }
    return dataFile;
}

/**
 * @brief Writes a header row with column legends to the given file.
 */
void setColumnLegends(File &file, int numAngles) {
    file.print("time,");
    file.print("aX,");
    file.print("aY,");
    file.print("aZ,");
    file.print("gX,");
    file.print("gY,");
    file.print("gZ,");
    file.print("T,");
    file.print("H,");
    file.print("P,");
    file.print("AQ,");

    for(int i=0; i<numAngles; i++){
        file.print("D" + String(i*10) + ",");
    }

    file.print("UTC,");
    file.print("Lat,");
    file.print("Long,");
    file.print("Speed_knots,");
    file.print("Speed_mph,");

    file.println();
}

/**
 * @brief Appends a row of data to the CSV file.
 */
void logData(File &file,
             unsigned long timestamp,
             float *receivedData,
             int numAngles,
             const int *minDistances,
             const String &utcTime,
             float latitude,
             float longitude,
             float speedKnots,
             float speedMph)
{
    if(file) {
        file.print(timestamp); file.print(",");
        file.print(receivedData[0], 2); file.print(",");
        file.print(receivedData[1], 2); file.print(",");
        file.print(receivedData[2], 2); file.print(",");
        file.print(receivedData[3], 2); file.print(",");
        file.print(receivedData[4], 2); file.print(",");
        file.print(receivedData[5], 2); file.print(",");
        file.print(receivedData[6], 2); file.print(",");
        file.print(receivedData[7], 2); file.print(",");
        file.print(receivedData[8], 2); file.print(",");
        file.print(receivedData[9], 2); file.print(",");

        for(int i=0; i<numAngles; i++){
            file.print(minDistances[i]);
            file.print(",");
        }

        file.print(utcTime); file.print(",");
        file.print(latitude,6); file.print(",");
        file.print(longitude,6); file.print(",");
        file.print(speedKnots,2); file.print(",");
        file.print(speedMph,2); file.print(",");

        file.println();
        file.flush();
    } else {
        Serial.println("Error: File not open for writing.");
    }
}
