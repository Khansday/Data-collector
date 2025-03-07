//~~~~~~~~~~~~~~~~~~~~~~PORTENTA PART~~~~~~~~~~~~~~~~~~~~~~~~~
#include <Wire.h>

#define PER_ADDR 0x55  // I2C address of Nicla
#define DATA_SIZE 40  // 10 floats (4 bytes each) for the sensor data

float receivedData[12];  // Array to store received sensor data

unsigned long previous_time = 0;
const unsigned long loop_frequency = 50000;  //in microseconds
float current_frequency;

void setup() {
    Wire.begin();  // Start I2C as master
    Serial.begin(250000);

    if (!checkI2CConnection()) {
        Serial.println("I2C device not found.");
        while (1);  // Stop execution if connection fails
    } else {
        Serial.println("I2C device connected successfully.");
    }
}

void loop() {
  unsigned long current_time = micros();
  unsigned long elapsed_time = current_time - previous_time;

  if (elapsed_time >= loop_frequency) { // Ensure the loop runs at approximately 1 kHz (1ms per loop)
    previous_time = current_time;
    double delta_time = double(elapsed_time) / 1000000.0f;
    current_frequency = 1 / delta_time; // just use for debugging
  
    requestSensorData();  // Request sensor data from Nicla
  }
}

// Function to request sensor data from Nicla and store it in receivedData array
void requestSensorData() {
    Wire.requestFrom(PER_ADDR, DATA_SIZE);

    if (Wire.available() == DATA_SIZE) {
        Wire.readBytes((char*)receivedData, DATA_SIZE);  // Read all sensor data into the array

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
        Serial.print(" F: "); Serial.println(current_frequency);
    } else {
        Serial.println("Error: Incomplete data received.");
    }
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
