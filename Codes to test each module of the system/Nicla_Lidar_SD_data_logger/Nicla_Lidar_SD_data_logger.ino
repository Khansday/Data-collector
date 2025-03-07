//~~~~~~~~~~~~~~~~~~~~~~PORTENTA SENSOR - LIDAr - LOGGER~~~~~~~~~~~~~~~~~~~~~~~~~
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Arduino.h>
#include "mbed.h"  // Include Mbed OS headers to apply mbed_app.json configurations


#define PER_ADDR 0x55  // I2C address of Nicla
#define DATA_SIZE 40  // 10 floats (4 bytes each) for the sensor data
#define NEW_FILE_TIME 30000  //milliseconds to create a new file
#define LOG_INTERVAL 50  // Time interval for logging data (50 ms for 20 Hz)
#define CHECK_CONNECTION_INTERVAL 5000
#define PARSE_LIDAR_INTERVAL 7

float current_log_frequency;
unsigned long long int timestamp{};

const int chipSelect = 7; // Define the CS pin for SD card
File dataFile;
int fileIndex = 1; // Start with the first file index

float receivedData[12]{};  // Array to store received sensor data

#define BUFFER_SIZE 10000 // Define the size of the buffer
byte raw_array[BUFFER_SIZE] = {0};  // Initialize the buffer
bool new_lidar_data_flag = false;
const int NUM_ANGLES = 19; // Represents angles from 0 to 170 in steps of 10 degrees
int min_distances[NUM_ANGLES];
unsigned long buffer_fill_time_limit = 5; // Time limit for filling the buffer
unsigned long parsing_time_limit = 3;     // Time limit for parsing the buffer

// Circular buffer indices and counters
volatile size_t read_index = 0;
volatile size_t write_index = 0;
volatile size_t buffer_count = 0;


bool I2C_connection_status = false;
bool SD_connection_status = false;
bool Lidar_connection_status = false;


void setup() {
  // Initialize Serial for debugging and I2C
  Serial.begin(250000);
  Wire.begin();  // Start I2C as master
  Serial1.begin(230400);   // Start Serial1 to communicate with the LiDAR module


  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  blinkLEDS();

  setYellow();  //start of the initialisation

  // Initialize SD card
  if (!SD.begin(chipSelect)) {
      Serial.println("SD Initialization failed!");
  }
  else{
      SD_connection_status = true;
      Serial.println("SD card initialization done.");
      createNewFile();  // Create the first file for logging
  }
  
  // Check I2C connection
  if (checkI2CConnection()) {
      I2C_connection_status = true;
      Serial.println("I2C device connected successfully.");
  } 
  else {
      Serial.println("I2C device not found.");
  }

  // Check Serial1 uart connection
  if (Serial1.available()){
    Lidar_connection_status = true;
    Serial.println("Lidar connected");
  }
  else {
      Serial.println("Lidar not found.");
  }
  
  init_distance_array();
  
  updateLEDs();

  delay(3000);
}

void loop() {
  static unsigned long previousFileMillis = 0;  // Timer for creating new file
  static unsigned long previousLogMillis = 0;   // Timer for logging data
  static unsigned long previousConnectionCheckMillis = 0; // Timer for conncection checks
  static unsigned long previousParseLidarMillis = 0; // Timer for conncection checks


  unsigned long currentMillis = millis();

  // Timer to check connections every 5 seconds
  if (currentMillis - previousConnectionCheckMillis >= CHECK_CONNECTION_INTERVAL) {
    previousConnectionCheckMillis = currentMillis;
    
    // Check SD card
    if (SD.begin(chipSelect)) {
      SD_connection_status = true;
      Serial.println("SD card is still connected.");
    } else {
        SD_connection_status = false;
        Serial.println("SD card disconnected or failed!");
    }

    // Check I2C connection
    if (quickPingI2C()) {
        I2C_connection_status = true;
        Serial.println("I2C device is still connected.");
    } else {
        I2C_connection_status = false; 
        Serial.println("I2C device disconnected or not responding!");
           
    }

    // Check I2C connection
    if (Serial1.available()) {
        Lidar_connection_status = true;
        Serial.println("Lidar is still connected.");
    } else {
        Lidar_connection_status = false; 
        Serial.println("Lidar disconnected or not responding!");
           
    }
    updateLEDs();
  }


  // Timer for creating a new file every NEW_FILE_TIME milliseconds (30 sec)
  if ((currentMillis - previousFileMillis >= NEW_FILE_TIME) & SD_connection_status ) { 
    previousFileMillis = currentMillis;
    createNewFile();
  }


  // Timer for logging data every LOG_INTERVAL milliseconds (20 Hz)
  if ((currentMillis - previousLogMillis >= LOG_INTERVAL) ) {
    unsigned long int elapsed_log_time = currentMillis - previousLogMillis;
    current_log_frequency = 1000.0 / float(elapsed_log_time) ; // just use for debugging
    previousLogMillis = currentMillis;
    //get motion and enviroment data only if available
    if (I2C_connection_status) requestSensorData();  // Request sensor data from the I2C bus
    //printRequestedData(); //for debugging
    if (dataFile.availableForWrite()) logData();  // Log the sensor data to the SD card
  }

  //time for parsing lidar data (7 ms)
  if ((currentMillis - previousParseLidarMillis >= PARSE_LIDAR_INTERVAL) & new_lidar_data_flag){
    parse_raw_buffer_min_angle();
    new_lidar_data_flag = false; // Reset the flag after parsing
  }


  // Fill the buffer with data from LiDAR for a specified time or until full
  full_raw_buffer();
}

// Function to request sensor data from Nicla and store it in receivedData array
void requestSensorData() {
  Wire.requestFrom(PER_ADDR, DATA_SIZE);

  if (Wire.available() == DATA_SIZE) {
      Wire.readBytes((char*)receivedData, DATA_SIZE);  // Read all sensor data into the array
  } else {
      Serial.println("Error: Incomplete I2C data received.");
  }
}

//Print data from Nicla for debug
void printRequestedData(){
  // Print the received data
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

// Function to check if the I2C device is connected
bool checkI2CConnection() {
    Wire.beginTransmission(PER_ADDR);
    byte error = Wire.endTransmission();

    if (error == 0) {
        return true;  // Connection successful
    } else {
        return false;  // Connection failed
    }
}

// Function to "ping" the I2C device and handle different outcomes
bool quickPingI2C() {
    const int bytesRequested = 1;  // Request just 1 byte for the ping
    int bytesReceived = Wire.requestFrom(PER_ADDR, bytesRequested);  // Request 1 byte from the I2C device

    // 3. Check for NACK (No Acknowledge) - if the device did not respond
    if (bytesReceived == 0) {
        Serial.println("I2C device did not acknowledge the request (NACK).");
        return false;  // Indicate the connection check failed
    }

    // 2. Check if data is available after the request
    if (Wire.available() == 0) {
        Serial.println("No data received from I2C device.");
        return false;  // Indicate the connection check failed
    }

    // 1. Handle default data - retrieve the received byte and print it
    byte receivedByte = Wire.read();
    Serial.print("Received byte: ");
    Serial.println(receivedByte, HEX);  // Print the received byte in hexadecimal format

    // Successful ping
    return true;  // Indicate the connection check was successful
}

// Function to create a new file on the SD card
void createNewFile() {
  if (dataFile) {
      dataFile.close();  // Close the current file if open
  }
  while(1){
  String fileName = "Data" + String(fileIndex) + ".csv";
  if (!SD.exists(fileName.c_str()) ) {
      dataFile = SD.open(fileName.c_str(), FILE_WRITE);  // Open a new file for writing
      if (dataFile) {
          //Serial.print("Created new file: ");
          //Serial.println(fileName);
          setColoumnLegends();
          fileIndex++;  // Increment the file index for next file
          break;
      } else {
          Serial.println("Error creating file");
      }
  } else {
      fileIndex++;  // Try the next file index if the file already exists
  }
  }
}

void setColoumnLegends(){
  dataFile.print("time"); dataFile.print(",");
  dataFile.print("aX"); dataFile.print(",");
  dataFile.print("aY"); dataFile.print(",");
  dataFile.print("aZ"); dataFile.print(",");
  dataFile.print("gX"); dataFile.print(",");
  dataFile.print("gY"); dataFile.print(",");
  dataFile.print("gZ"); dataFile.print(",");
  dataFile.print("T"); dataFile.print(",");  
  dataFile.print("H"); dataFile.print(","); 
  dataFile.print("P"); dataFile.print(",");  
  dataFile.print("AQ"); dataFile.print(","); 
  for (int idx = 0; idx < NUM_ANGLES; idx++) {
    String Lidar_data = " D" + String(idx*10) + "," ;
    dataFile.print(Lidar_data);
  }
  dataFile.println();
}

// Function to log the sensor data to the SD card
void logData() {
    if (dataFile) {
        timestamp = millis();
        // Log the sensor data in a readable format
        dataFile.print(timestamp); dataFile.print(",");
        
        dataFile.print(receivedData[0], 2); dataFile.print(",");
        dataFile.print(receivedData[1], 2); dataFile.print(",");
        dataFile.print(receivedData[2], 2); dataFile.print(",");

        dataFile.print(receivedData[3], 2); dataFile.print(","); 
        dataFile.print(receivedData[4], 2); dataFile.print(",");
        dataFile.print(receivedData[5], 2); dataFile.print(",");

        dataFile.print(receivedData[6], 2); dataFile.print(","); 
        dataFile.print(receivedData[7], 2); dataFile.print(","); 
        dataFile.print(receivedData[8], 2); dataFile.print(","); 
        dataFile.print(receivedData[9], 2);dataFile.print(","); 

        for (int idx = 0; idx < NUM_ANGLES; idx++) {
          String Lidar_data = String(min_distances[idx]) +",";
          dataFile.print(Lidar_data);
        }

        dataFile.println();

        //Serial.println("Data written to file.");  // Optional: For debugging
        dataFile.flush();  // Ensure the data is written to the SD card
    } else {
        //Serial.println("Error writing to file");
    }
}

// Function to start filling raw Lidar data circularly
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
          break;
        }
      }
    }
  }

  if (millis() - start_time >= buffer_fill_time_limit) {
    //Serial.println("Buffer overtime");
  }
}

//Function to retrive the min value form a packet and update main distance array
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
        if (start_angle_approx >= 20 && start_angle_approx < 185) {
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
    //Serial.println("Parsing time limit reached");
  }
}

void init_distance_array(){
  // Initialize min_distances with a large value
  for (int idx = 0; idx < NUM_ANGLES; idx++) {
    min_distances[idx] = 65535;
  }
}

void updateLEDs(){
  if (SD_connection_status & I2C_connection_status & Lidar_connection_status) setWhite(); 
  else if (!SD_connection_status & !I2C_connection_status & !SD_connection_status) setRed();
  else if (SD_connection_status & !I2C_connection_status & Lidar_connection_status) setCyan();
  else if (!SD_connection_status & I2C_connection_status & Lidar_connection_status) setMagenta();
  else if (!Lidar_connection_status) setYellow();
  // Serial.print("SD ");
  // Serial.print(SD_connection_status);
  // Serial.print("I2C ");
  // Serial.println(I2C_connection_status);
  // Serial.print("Lidar ");
  // Serial.println(Lidar_connection_status);
}

void blinkLEDS(){
  turnOffLED();
  setRed();
  delay(50);  // Wait for 50 milliseconds
  
  setGreen();
  delay(50);
  
  setBlue();
  delay(50);

  turnOffLED();
  delay(100); 
}

// Functions to set different colors
void turnOffLED() {  // Black (all LEDs off)
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}

void setRed() {  // Red
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}

void setGreen() {  // Green
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, HIGH);
}

void setBlue() {  // Blue
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, LOW);
}

void setCyan() {  // Cyan (Green + Blue)
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);
}

void setMagenta() {  // Magenta (Red + Blue)
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, LOW);
}

void setYellow() {  // Yellow (Red + Green)
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, HIGH);
}

void setWhite() {  // White (Red + Green + Blue)
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);
}
