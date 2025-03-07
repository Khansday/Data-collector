//~~~~~~~~~~~~~~NICLA PART~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//#####   NOTE!! : THE SENDING OF THE ATTITUDE DATA IN THE I2C HAS TO BE IMPLEMENTED#################### 
#include <Wire.h>
#include "Arduino_BHY2.h"  // Include sensor library for Nicla Sense ME
#include "Nicla_System.h"

//#define PER_ADDR 0x10  // I2C address for the Nicla  OVERRIDDEN!!!!

// Predefined array to hold sensor data
// Adjusted size to match the number of sensor values being sent (10)
float sensorData[10];  // 3 for accelerometer, 3 for gyroscope, 1 for temperature, 1 for humidity, 1 for pressure, and 1 for air quality

// Initialize sensor objects with updated sensor IDs and correct types
SensorXYZ accel(SENSOR_ID_ACC);             // Accelerometer
SensorXYZ gyro(SENSOR_ID_GYRO);             // Gyroscope
Sensor temperature(SENSOR_ID_TEMP_WU);      // Temperature with wake-up functionality
Sensor humidity(SENSOR_ID_HUM);             // Humidity
Sensor pressure(SENSOR_ID_BARO);            // Pressure
SensorBSEC gas(SENSOR_ID_BSEC);              // Air quality sensor (BSEC)
SensorOrientation attitude (SENSOR_ID_ORI);   //Device orientation Euler
 
//THE FOLLWING ARE NEEDED TO CONVERT RAW VALUES TO STANDARD
#define ACC_SCALE_FACTOR 4096.0
#define GYRO_SCALE_FACTOR 131.2

void setup() {
  Serial.begin(250000);  // For debugging purposes

  nicla::begin();
  nicla::leds.begin();
  nicla::leds.setColor(yellow);

  // Initialize BHY2 and all sensors
  if (!BHY2.begin()) {
      Serial.println("Failed to initialize BHY2!");
      nicla::leds.setColor(red);
      while (1);
  }

  Wire.onRequest(requestEvent);  // Register the function that sends data to master

  accel.begin();
  gyro.begin();
  temperature.begin();
  humidity.begin();
  pressure.begin();
  gas.begin();
  attitude.begin();

  Serial.println("Program Started!");
  nicla::leds.setColor(green);
}

void loop() {
  static auto lastCheck= millis();
  BHY2.update();  // Keep sensor data updated
  // Check sensor values every second  
  if (millis() - lastCheck >= 10) {  //Print data for debug
    lastCheck = millis();
    //prepareSensorData();
    //printSensorData();
    //printAccGyroData();
    //printAttitudeData();
  }
  delay(1);       // Add small delay to avoid flooding the I2C bus
}

// Function to gather sensor data and prepare it for sending
void prepareSensorData() {
    // Access accelerometer data directly from the accel object
    sensorData[0] = accel.x() / ACC_SCALE_FACTOR;  // X-axis acceleration 
    sensorData[1] = accel.y() / ACC_SCALE_FACTOR;  // Y-axis acceleration 
    sensorData[2] = accel.z() / ACC_SCALE_FACTOR;  // Z-axis acceleration 

    // Access gyroscope data directly from the gyro object
    sensorData[3] = gyro.x() / GYRO_SCALE_FACTOR;    // X-axis angular velocity 
    sensorData[4] = gyro.y() / GYRO_SCALE_FACTOR;    // Y-axis angular velocity 
    sensorData[5] = gyro.z() / GYRO_SCALE_FACTOR;    // Z-axis angular velocity 

    // Access scalar sensor data
    sensorData[6] = temperature.value() -9; // Temperature in °C
    sensorData[7] = humidity.value();    // Humidity in %
    sensorData[8] = pressure.value();    // Pressure in hPa
    sensorData[9] = gas.iaq();         // Air Quality Index (IAQ)
}

// Function to handle I2C requests from the master
void requestEvent() {
    prepareSensorData();  // Update the sensor data before sending

    Wire.write((byte*)sensorData, sizeof(sensorData));  // Send all sensor data
    Serial.println("Sensor data sent");
}

//Debug print all data
void printSensorData(){
    Serial.print("A: ");
    Serial.print(sensorData[0] ); Serial.print(", ");
    Serial.print(sensorData[1] ); Serial.print(", ");
    Serial.print(sensorData[2] ); Serial.print(" | ");

    Serial.print("G: ");
    Serial.print(sensorData[3] ); Serial.print(", ");
    Serial.print(sensorData[4] ); Serial.print(", ");
    Serial.print(sensorData[5] 
    ); Serial.print(" | ");

    Serial.print("T: ");
    Serial.print(sensorData[6]); Serial.print(" °C | ");

    Serial.print("H: ");
    Serial.print(sensorData[7]); Serial.print(" % | ");

    Serial.print("P: ");
    Serial.print(sensorData[8]); Serial.print(" hPa | ");

    Serial.print("AQ: ");
    Serial.println(sensorData[9]);

}

//Debug print acc and gyro data
void printAccGyroData(){

  // Print Accelerometer data
  Serial.print("A"); Serial.print(", ");
  Serial.print(sensorData[0] ); Serial.print(", ");
  Serial.print(sensorData[1] ); Serial.print(", ");
  Serial.print(sensorData[2] ); Serial.print(", ");

  // Print Gyroscope data
  Serial.print("G"); Serial.print(", ");
  Serial.print(sensorData[3] ); Serial.print(", ");
  Serial.print(sensorData[4] ); Serial.print(", ");
  Serial.println(sensorData[5] );  // Use Serial.println for final value to end the line

}

//Debug print roll pitch and heading
void printAttitudeData(){
  // Print Accelerometer data
  Serial.print("Roll: "); Serial.print(attitude.roll());
  Serial.print("Pitch: "); Serial.print(attitude.pitch());
  Serial.print("Heading: "); Serial.println(attitude.heading());
}
