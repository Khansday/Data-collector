#include <SPI.h>
#include <SD.h>
#include <Wire.h>

const int chipSelect = 7; // Define the CS pin, change if needed
File dataFile;
int fileIndex = 1; // Start with the first file index
#define NEW_FILE_TIME 10000  //milliseconds to make a new file 

void setup() {
  Serial.begin(250000);
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("Initialization failed!");
    while (1);
  }
  Serial.println("Initialization done.");

 
  createNewFile();
}

void loop() {
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= NEW_FILE_TIME) { 
    previousMillis = currentMillis;
    createNewFile();
  }

  logData();
  delay(1000); // Adjust this delay as per the requirement of data recording frequency
}

void createNewFile() {
  if (dataFile) {
    dataFile.close();
  }

  while (true) {
    String fileName = "Data" + String(fileIndex) + ".txt";
    if (!SD.exists(fileName.c_str())) {
      dataFile = SD.open(fileName.c_str(), FILE_WRITE);
      if (dataFile) {
        Serial.print("Created new file: ");
        Serial.println(fileName);
        fileIndex++;
        break;
      } else {
        Serial.println("Error creating file");
      }
    } else {
      fileIndex++; // Increment the file index and try again
    }
  }
}

void logData() {
  if (dataFile) {

    dataFile.print("Heloooo");
    dataFile.println();

    Serial.println("Data written to file.");
    dataFile.flush(); // Ensure data is written to the SD card
  } else {
    Serial.println("Error writing to file");
  }
}
