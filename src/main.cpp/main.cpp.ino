#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "mbed.h"

// Our module headers
#include "LEDControl.h"
#include "SDManager.h"
#include "GPSParser.h"
#include "LidarProcessor.h"

// ========== Definitions and Globals ==========
#define PER_ADDR 0x55      // I2C address for Nicla
#define DATA_SIZE 40       // 10 floats (4 bytes each)
#define NEW_FILE_TIME 30000
#define LOG_INTERVAL 50
#define CHECK_CONNECTION_INTERVAL 5000
#define RECONNECT_DEVICES_INTERVAL 6000
#define DEVICE_RECONNECT_TIMEOUT 200
#define PARSE_LIDAR_INTERVAL 7
#define PARSE_GPS_INTERVAL 1000
#define UPDATE_LED_INTERVAL 500

// Chip select for SD
const int chipSelect = 7;

// I2C
bool I2C_connection_status   = false;
bool SD_connection_status    = false;
bool Lidar_connection_status = false;
bool GPS_connection_status   = true;  // TEMP assume is fine

// For GPS
UART UART_GPS(PC_6, PC_7); // Using UART6 pins

// Store sensor data from Nicla
float receivedData[12] = {0};

// Frequency tracking
float current_log_frequency;
unsigned long long int timestamp;

// File management
File dataFile;
int fileIndex = 1; // Start with first file

// Timers
static unsigned long previousFileMillis             = 0;
static unsigned long previousLogMillis              = 0;
static unsigned long previousConnectionCheckMillis  = 0;
static unsigned long previousReconnectDevicesMillis = 0;
static unsigned long previousParseLidarMillis       = 0;
static unsigned long previousGPSMillis              = 0;
static unsigned long previousLEDMillis              = 0;

// ========== Function Prototypes (only those that remain in main) ==========
bool checkI2CConnection();
bool quickPingI2C();
void requestSensorData();
void printRequestedData();  // for debugging if needed

// ========== SETUP ==========
void setup() {
    Serial.begin(250000);
    Wire.begin();  
    Serial1.begin(230400);   // for LiDAR
    UART_GPS.begin(9600);  

    initLEDs();
    blinkAllLEDs();  // quick check

    // Attempt to set GPS to 115200
    setGPSBaudRate(UART_GPS, 115200);
    // Reinit at new baud
    UART_GPS.begin(115200);

    delay(500); //for stability

    // Check SD
    SD_connection_status = initSDCard(chipSelect);
    if(SD_connection_status) {
        dataFile = createNewFile(fileIndex);
        setColumnLegends(dataFile, NUM_ANGLES);
    }

    // Check I2C
    if(checkI2CConnection()) {
        I2C_connection_status = true;
        Serial.println("I2C device connected successfully.");
    } else {
        Serial.println("I2C device not found.");
    }

    // Check LiDAR
    if(Serial1.available()) {
        Lidar_connection_status = true;
        Serial.println("Lidar connected.");
    } else {
        Serial.println("Lidar not found.");
    }

    // Check GPS
    if(UART_GPS.available() > 0) {
        GPS_connection_status = true;
        Serial.println("GPS connected.");
    } else {
        Serial.println("GPS not found.");
    }

    initDistanceArray();
    delay(500);
}

// ========== LOOP ==========
void loop() {
    unsigned long currentMillis = millis();

    // 1) Check connections every 5s
    if(currentMillis - previousConnectionCheckMillis >= CHECK_CONNECTION_INTERVAL) {
        previousConnectionCheckMillis = currentMillis;

        // Re-check SD
        SD_connection_status = SD.begin(chipSelect);
        if(SD_connection_status) Serial.println("SD card OK.");
        else                     Serial.println("SD card disconnected.");

        // Re-check I2C
        if(quickPingI2C()) {
            I2C_connection_status = true;
            Serial.println("I2C OK.");
        } else {
            I2C_connection_status = false;
            Serial.println("I2C disconnected.");
        }

        // Re-check LiDAR
        if(Serial1.available()) {
            Lidar_connection_status = true;
            Serial.println("Lidar OK.");
        } else {
            Lidar_connection_status = false;
            Serial.println("Lidar disconnected.");
        }
        // (GPS check omitted for brevity) TO BE ADDED
    }

    // 2) Try reconnecting devices every 6s
    if(currentMillis - previousReconnectDevicesMillis >= RECONNECT_DEVICES_INTERVAL) {
        previousReconnectDevicesMillis = currentMillis;
        unsigned long timeout_time = millis();

        // Reconnect SD
        while(!SD_connection_status && (millis() - timeout_time < DEVICE_RECONNECT_TIMEOUT)) {
            SD_connection_status = SD.begin(chipSelect);
        }

        // Reconnect I2C
        timeout_time = millis();
        while(!I2C_connection_status && (millis() - timeout_time < DEVICE_RECONNECT_TIMEOUT)) {
            Wire.begin();
            I2C_connection_status = checkI2CConnection();
        }

        // Reconnect Lidar
        timeout_time = millis();
        while(!Lidar_connection_status && (millis() - timeout_time < DEVICE_RECONNECT_TIMEOUT)) {
            Serial1.begin(230400);
            Lidar_connection_status = (Serial1.available() > 0);
        }
        // Reconnect GPS if needed...  TO BE ADDED
    }

    // 3) Create a new file every NEW_FILE_TIME if SD is OK
    if((currentMillis - previousFileMillis >= NEW_FILE_TIME) && SD_connection_status) {
        previousFileMillis = currentMillis;
        if(dataFile) dataFile.close();
        dataFile = createNewFile(fileIndex);
        setColumnLegends(dataFile, NUM_ANGLES);
    }

    // 4) Log data every LOG_INTERVAL
    if(currentMillis - previousLogMillis >= LOG_INTERVAL) {
        unsigned long elapsed = currentMillis - previousLogMillis;
        current_log_frequency = 1000.0 / float(elapsed);
        previousLogMillis     = currentMillis;

        // If I2C OK, request data
        if(I2C_connection_status) {
            requestSensorData();
            // printRequestedData();
        }

        // Log data
        if(SD_connection_status && dataFile) {
            timestamp = millis();
            logData(dataFile, 
                    timestamp, 
                    receivedData, 
                    NUM_ANGLES, 
                    min_distances, 
                    utcTime, 
                    latitude, 
                    longitude, 
                    speed_knots, 
                    speed_mph);
        }
    }

    // 5) Parse Lidar data every PARSE_LIDAR_INTERVAL
    if(currentMillis - previousParseLidarMillis >= PARSE_LIDAR_INTERVAL) {
        previousParseLidarMillis = currentMillis;
        parseBuffer(3); // parse for up to 3 ms (your original 'parsing_time_limit')
    }

    // 6) Parse GPS every PARSE_GPS_INTERVAL
    if(currentMillis - previousGPSMillis >= PARSE_GPS_INTERVAL) {
        previousGPSMillis = currentMillis;
        GPSParser(UART_GPS, 800); // PORT ,time limit for function
    }

    // 7) Update LED color(s) every UPDATE_LED_INTERVAL
    if(currentMillis - previousLEDMillis >= UPDATE_LED_INTERVAL) {
        previousLEDMillis = currentMillis;
        updateLEDs(I2C_connection_status,
                   SD_connection_status,
                   Lidar_connection_status,
                   GPS_connection_status);
    }

    // 8) Fill LiDAR buffer quickly each loop with a short limit (5 ms)
    fillBuffer(Serial1, 5);
}

// ========== Implementation of any leftover local functions ==========

bool checkI2CConnection() {
    Wire.beginTransmission(PER_ADDR);
    byte error = Wire.endTransmission();
    return (error == 0);
}

bool quickPingI2C() {
    const int bytesRequested = 1;
    int bytesReceived = Wire.requestFrom(PER_ADDR, bytesRequested);
    if(bytesReceived == 0) {
        return false;
    }
    if(Wire.available() == 0) {
        return false;
    }
    // read the byte (not used here)
    Wire.read();
    return true;
}

void requestSensorData() {
    Wire.requestFrom(PER_ADDR, DATA_SIZE);
    if(Wire.available() == DATA_SIZE) {
        Wire.readBytes((char*)receivedData, DATA_SIZE);
    } else {
        // Handle incomplete data
    }
}

void printRequestedData() {
    Serial.print("A: ");
    Serial.print(receivedData[0], 2); Serial.print(", ");
    Serial.print(receivedData[1], 2); Serial.print(", ");
    Serial.print(receivedData[2], 2);

    Serial.print(" G: ");
    Serial.print(receivedData[3], 2); Serial.print(", ");
    Serial.print(receivedData[4], 2); Serial.print(", ");
    Serial.print(receivedData[5], 2);

    Serial.print(" T: "); Serial.print(receivedData[6], 2);
    Serial.print(" H: "); Serial.print(receivedData[7], 2);
    Serial.print(" P: "); Serial.print(receivedData[8], 2);
    Serial.print(" AQ: "); Serial.print(receivedData[9], 2);
    Serial.print(" F: "); Serial.println(current_log_frequency);
}
