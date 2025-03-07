#include "Serial.h"
UART UART_GPS = UART(PC_6, PC_7);  // Using UART6 PINS

// Define RGB LED pins
#define LED_RED   LEDR
#define LED_GREEN LEDG
#define LED_BLUE  LEDB

// Define latitude, longitude, and time as global variables
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
  Serial.println("Program started");

  UART_GPS.begin(9600);  // Start communication at default baud rate (9600 bps)

  // Change GPS module baud rate to 115200 bps
  SAMM10QsetGPSBaudRate(115200);

  // Reinitialize UART_GPS with the new baud rate
  UART_GPS.begin(115200);
}

void loop() {
  unsigned long int timer = millis();
  while (UART_GPS.available() > 0) {  // Check for GPS data
    String nmeaData = UART_GPS.readStringUntil('\n');
    //Serial.println(nmeaData);

    // Check if the data is a GNGGA or GPGGA sentence
    if (nmeaData.startsWith("$GNGGA") || nmeaData.startsWith("$GPGGA")) {
      parseGGA(nmeaData);
      printLatLon();
      //Serial.println(millis() - timer); //WITHOUT PRINTING THIS ALL TAKES 8 ms
    }
    else if (nmeaData.startsWith("$GNRMC") || nmeaData.startsWith("$GPRMC")) {
      parseRMC(nmeaData);
      printData();
    }
  }
}

void SIM28setGPSBaudRate(uint32_t newBaudRate) {
  // Construct the command to change baud rate
  String command = "$PMTK251," + String(newBaudRate);

  // Calculate checksum
  byte checksum = 0;
  for (int i = 1; i < command.length(); i++) {
    checksum ^= command[i];
  }

  // Append checksum to command
  command += "*";
  if (checksum < 16) command += "0";  // Ensure two-digit checksum
  command += String(checksum, HEX);
  command += "\r\n";

  // Send command to GPS module
  UART_GPS.print(command);

  // Wait for the GPS module to process the command
  delay(100);

  // Flush any remaining data
  while (UART_GPS.available() > 0) {
    UART_GPS.read();
  }
}

void SAMM10QsetGPSBaudRate(uint32_t newBaudRate) {
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

  delay(100);  // Allow sufficient time for configuration

  // Flush input buffer
  while (UART_GPS.available()) {
    UART_GPS.read();
  }

  // Switch to new baud rate
  UART_GPS.end();
  delay(100);
  UART_GPS.begin(newBaudRate);
}

void parseGGA(String sentence) {  // GPS parsing function
  int commaIndex;
  int startIndex = 0;
  String fields[15];  // GGA has up to 15 fields

  // Split the NMEA sentence into fields
  for (int i = 0; i < 15; i++) {
    commaIndex = sentence.indexOf(',', startIndex);
    if (commaIndex == -1) {
      fields[i] = sentence.substring(startIndex);
      break;
    }
    fields[i] = sentence.substring(startIndex, commaIndex);
    startIndex = commaIndex + 1;
  }

  // Extract time
  utcTime = formatTime(fields[1]);

  // Check if it's a GGA sentence and has enough fields for latitude and longitude
  if ((fields[0] == "$GNGGA" || fields[0] == "$GPGGA") && fields[6].toInt() > 0) {
    // Only proceed if fix quality is greater than 0 (valid fix)
    if (fields[2].length() >= 4 && fields[4].length() >= 5) {
      
      // Extract latitude
      latitude = fields[2].substring(0, 2).toDouble() + fields[2].substring(2).toDouble() / 60.0;
      if (fields[3] == "S") {
        latitude = -latitude;
      }

      // Extract longitude
      longitude = fields[4].substring(0, 3).toDouble() + fields[4].substring(3).toDouble() / 60.0;
      if (fields[5] == "W") {
        longitude = -longitude;
      }
      setGreen();
    }
  } else {
    // Handle no fix case
    //Serial.println("No GPS fix");
    setRed();
  }
}

void parseRMC(String sentence) {  // GPS parsing function for RMC sentences
  int commaIndex;
  int startIndex = 0;
  String fields[13];  // RMC has up to 13 fields

  // Split the NMEA sentence into fields
  for (int i = 0; i < 13; i++) {
    commaIndex = sentence.indexOf(',', startIndex);
    if (commaIndex == -1) {
      fields[i] = sentence.substring(startIndex);
      break;
    }
    fields[i] = sentence.substring(startIndex, commaIndex);
    startIndex = commaIndex + 1;
  }

  // Check if status is 'A' (Valid)
  if (fields[2] == "A") {
    // Extract time
    utcTime = formatTime(fields[1]);

    // Extract latitude
    if (fields[3].length() >= 4) {
      latitude = fields[3].substring(0, 2).toDouble() + fields[3].substring(2).toDouble() / 60.0;
      if (fields[4] == "S") {
        latitude = -latitude;
      }
    }

    // Extract longitude
    if (fields[5].length() >= 5) {
      longitude = fields[5].substring(0, 3).toDouble() + fields[5].substring(3).toDouble() / 60.0;
      if (fields[6] == "W") {
        longitude = -longitude;
      }
    }

    // Extract speed in knots
    speed_knots = fields[7].toFloat();
    // Convert to km/h
    speed_kmh = speed_knots * 1.852;

    setGreen();
  } else {
    // Handle invalid data
    setRed();
  }
}

void printLatLon() {
  Serial.print("Time: ");
  Serial.print(utcTime);
  Serial.print(" Lat: ");
  Serial.print(latitude, 6);  // Printing with precision
  Serial.print(" Lon: ");
  Serial.println(longitude, 6);  // Printing with precision
}

void printData() {
  Serial.print("Time: ");
  Serial.print(utcTime);
  Serial.print(" Lat: ");
  Serial.print(latitude, 6);  // Printing with precision
  Serial.print(" Lon: ");
  Serial.print(longitude, 6);  // Printing with precision
  Serial.print(" Speed: ");
  Serial.print(speed_knots, 2);
  Serial.print(" knots (");
  Serial.print(speed_kmh, 2);
  Serial.println(" km/h)");
}

String formatTime(String timeStr) {
  if (timeStr.length() < 6) {
    return "Invalid Time";
  }
  String hours = timeStr.substring(0, 2);
  String minutes = timeStr.substring(2, 4);
  String seconds = timeStr.substring(4, 6);

  // Optional: Convert to integer values if needed
  // int hourInt = hours.toInt();
  // int minuteInt = minutes.toInt();
  // float secondFloat = seconds.toFloat();

  return hours + ":" + minutes + ":" + seconds;
}

void setGreen() {  // Green
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, HIGH);
}

void setRed() {  // Red
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}
