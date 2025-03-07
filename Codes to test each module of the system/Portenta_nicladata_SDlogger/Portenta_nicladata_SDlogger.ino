//~~~~~~~~~~~~~~~~~~~~~~PORTENTA SENSOR LOGGER~~~~~~~~~~~~~~~~~~~~~~~~~
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#define PER_ADDR 0x55  // I2C address of Nicla
#define DATA_SIZE 40  // 10 floats (4 bytes each) for the sensor data
#define NEW_FILE_TIME 30000  //milliseconds to create a new file
#define LOG_INTERVAL 50  // Time interval for logging data (50 ms for 20 Hz)
#define CHECK_CONNECTION_INTERVAL 5000
float current_log_frequency;

const int chipSelect = 7; // Define the CS pin for SD card
File dataFile;
int fileIndex = 1; // Start with the first file index

float receivedData[12]{};  // Array to store received sensor data

bool I2C_connection_status = false;
bool SD_connection_status = false;

unsigned long long int timestamp{};


void setup() {
  // Initialize Serial for debugging and I2C
  Serial.begin(250000);
  Wire.begin();  // Start I2C as master

  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  blinkLEDS();

  setYellow();  //start of the initialisation

  // Initialize SD card
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
      Serial.println("Initialization failed!");
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

  
  updateLEDs();

  delay(3000);
}

void loop() {
  static unsigned long previousFileMillis = 0;  // Timer for creating new file
  static unsigned long previousLogMillis = 0;   // Timer for logging data
  static unsigned long previousConnectionCheckMillis =0; // Timer for conncection checks

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
    updateLEDs();
  }

  // Timer for creating a new file every NEW_FILE_TIME milliseconds (30 sec)
  if ((currentMillis - previousFileMillis >= NEW_FILE_TIME) & SD_connection_status ) { 
    previousFileMillis = currentMillis;
    createNewFile();
  }

  // Timer for logging data every LOG_INTERVAL milliseconds (20 Hz)
  if ((currentMillis - previousLogMillis >= LOG_INTERVAL & I2C_connection_status) ) {
    unsigned long int elapsed_log_time = currentMillis - previousLogMillis;
    previousLogMillis = currentMillis;
    current_log_frequency = 1000.0 / float(elapsed_log_time) ; // just use for debugging
    requestSensorData();  // Request sensor data from the I2C bus
    printRequestedData(); //for debugging
    if (dataFile.availableForWrite()) logData();  // Log the sensor data to the SD card
  }

}

// Function to request sensor data from Nicla and store it in receivedData array
void requestSensorData() {
  Wire.requestFrom(PER_ADDR, DATA_SIZE);

  if (Wire.available() == DATA_SIZE) {
      Wire.readBytes((char*)receivedData, DATA_SIZE);  // Read all sensor data into the array
  } else {
      Serial.println("Error: Incomplete data received.");
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
  String fileName = "Data" + String(fileIndex) + ".txt";
  if (!SD.exists(fileName.c_str()) ) {
      dataFile = SD.open(fileName.c_str(), FILE_WRITE);  // Open a new file for writing
      if (dataFile) {
          Serial.print("Created new file: ");
          Serial.println(fileName);
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

// Function to log the sensor data to the SD card
void logData() {
    if (dataFile) {
        timestamp = millis();
        // Log the sensor data in a readable format
        dataFile.print("time"); dataFile.print(",");
        dataFile.print(timestamp); dataFile.print(",");
        
        dataFile.print("A"); dataFile.print(",");
        dataFile.print(receivedData[0], 2); dataFile.print(",");
        dataFile.print(receivedData[1], 2); dataFile.print(",");
        dataFile.print(receivedData[2], 2); dataFile.print(",");

        dataFile.print("G"); dataFile.print(",");
        dataFile.print(receivedData[3], 2); dataFile.print(","); 
        dataFile.print(receivedData[4], 2); dataFile.print(",");
        dataFile.print(receivedData[5], 2); dataFile.print(",");

        dataFile.print(" T: "); dataFile.print(","); dataFile.print(receivedData[6], 2); dataFile.print(","); 
        dataFile.print(" H: "); dataFile.print(","); dataFile.print(receivedData[7], 2); dataFile.print(","); 
        dataFile.print(" P: "); dataFile.print(","); dataFile.print(receivedData[8], 2); dataFile.print(","); 
        dataFile.print(" AQ: "); dataFile.print(","); dataFile.println(receivedData[9], 2);
        dataFile.println();

        Serial.println("Data written to file.");  // Optional: For debugging
        dataFile.flush();  // Ensure the data is written to the SD card
    } else {
        Serial.println("Error writing to file");
    }
}

void updateLEDs(){
  if (SD_connection_status & I2C_connection_status) setGreen(); 
  else if (!SD_connection_status & !I2C_connection_status) setRed();
  else if (SD_connection_status & !I2C_connection_status) setCyan();
  else if (!SD_connection_status & I2C_connection_status) setMagenta();
  // Serial.print("SD ");
  // Serial.print(SD_connection_status);
  // Serial.print("I2C ");
  // Serial.println(I2C_connection_status);
}

void blinkLEDS(){
  turnOffLED();
  setRed();
  delay(50);  // Wait for 500 milliseconds
  
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
