// Include necessary libraries
#include "Serial.h"
UART UART_GPS = UART(PC_6, PC_7);  // Using UART6 PINS

// Define RGB LED pins
#define LED_RED   LEDR
#define LED_GREEN LEDG
#define LED_BLUE  LEDB

// Define latitude, longitude, time, and speed as global variables
float latitude = 0.0;
float longitude = 0.0;
String utcTime = "";
float speed_knots = 0.0;
float speed_kmh = 0.0;

void setup() {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  Serial.begin(250000);  // Debugging output
  delay(1000);
  Serial.println("Program started");

  UART_GPS.begin(9600);  // Start communication at default baud rate (9600 bps)
  delay(500);

  // Change GPS module baud rate to 115200 bps
  if (setGPSBaudRate(115200)) {
    Serial.println("Baud rate successfully changed to 115200");
    UART_GPS.begin(115200);  // Reinitialize UART_GPS with the new baud rate
  } else {
    Serial.println("Failed to change baud rate");
    setRed();
  }
}

void loop() {
  while (UART_GPS.available() > 0) {  // Check for GPS data
    String nmeaData = UART_GPS.readStringUntil('\n');
    Serial.println(nmeaData);

    // Check if the data is a GNGGA or GPGGA sentence
    if (nmeaData.startsWith("$GNGGA") || nmeaData.startsWith("$GPGGA")) {
      parseGGA(nmeaData);
      printData();
    }
    // Check if the data is a GNRMC or GPRMC sentence
    else if (nmeaData.startsWith("$GNRMC") || nmeaData.startsWith("$GPRMC")) {
      parseRMC(nmeaData);
      printData();
    }
  }
}

bool setGPSBaudRate(uint32_t newBaudRate) {
  // Construct PUBX,41 command to set baud rate
  String pubxMessage = "$PUBX,41,1,0007,0003," + String(newBaudRate) + ",0*";

  // Calculate checksum
  uint8_t checksum = 0;
  for (int i = 1; i < pubxMessage.length() - 1; i++) {
    checksum ^= pubxMessage[i];  // XOR each character
  }

  // Convert checksum to uppercase hexadecimal and append
  char checksumStr[3];  // 2 hex digits + null terminator
  snprintf(checksumStr, sizeof(checksumStr), "%02X", checksum);
  pubxMessage += checksumStr;
  pubxMessage += "\r\n";  // Append CRLF

  // Send PUBX message to GPS module
  UART_GPS.print(pubxMessage);

  delay(500);  // Allow sufficient time for configuration

  // Flush input buffer
  while (UART_GPS.available()) {
    UART_GPS.read();
  }

  // Switch to new baud rate
  UART_GPS.end();
  delay(100);
  UART_GPS.begin(newBaudRate);


  // Check if the module responds
  for (size_t i = 0; i < 500; i++) {
    if (UART_GPS.available()) {
      while (UART_GPS.available()) {
        UART_GPS.read();  // Clear response buffer
      }
      return true;  // Baud rate change successful
    }
    delay(1);
  }

  return false;  // Baud rate change failed
}


void parseGGA(String sentence) {
  // Implementation of GGA parsing
}

void parseRMC(String sentence) {
  // Implementation of RMC parsing
}

void printData() {
  // Implementation of data printing
}

void setGreen() {  // Green LED indicator
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, HIGH);
}

void setRed() {  // Red LED indicator
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}
