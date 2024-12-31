#include "GPSParser.h"

// Globals for storing GPS data
float latitude   = 0.0;
float longitude  = 0.0;
float speed_knots = 0.0;
float speed_mph   = 0.0;
String utcTime    = "";

static String formatTime(const String &timeStr) {
    if (timeStr.length() < 6) {
        return "Invalid Time";
    }
    String hours   = timeStr.substring(0, 2);
    String minutes = timeStr.substring(2, 4);
    String seconds = timeStr.substring(4, 6);
    return hours + ":" + minutes + ":" + seconds;
}

void initGPS(UART &uartGPS, unsigned long initialBaud) {
    // Start the GPS at some baud
    uartGPS.begin(initialBaud);
    // Additional config if needed
}

void setGPSBaudRate(UART &uartGPS, uint32_t newBaudRate) {
    // Example: Using the $PUBX,41,1... command from your code
    String pubxMessage = "$PUBX,41,1,0007,0003," + String(newBaudRate) + ",0*";
    uint8_t checksum = 0;
    for (int i = 1; i < pubxMessage.length() - 1; i++) {
        checksum ^= pubxMessage[i];
    }
    char checksumStr[3];
    snprintf(checksumStr, sizeof(checksumStr), "%02X", checksum);
    pubxMessage += checksumStr;
    pubxMessage += "\r\n";

    // Send
    uartGPS.print(pubxMessage);
    delay(100);

    // Flush
    while (uartGPS.available()) {
        uartGPS.read();
    }

    // Switch baud
    uartGPS.end();
    delay(100);
    uartGPS.begin(newBaudRate);
}

void GPSParser(UART &uartGPS, unsigned long timeLimit) {
    unsigned long start_time = millis();
    while ((millis() - start_time < timeLimit) && (uartGPS.available() > 0)) {
        String nmeaData = uartGPS.readStringUntil('\n');
        // Check for GGA
        if(nmeaData.startsWith("$GNGGA") || nmeaData.startsWith("$GPGGA")) {
            parseGGA(nmeaData);
        }
        // Check for RMC
        else if(nmeaData.startsWith("$GNRMC") || nmeaData.startsWith("$GPRMC")) {
            parseRMC(nmeaData);
        }
    }
}

void parseGGA(const String &sentence) {
    // GGA has up to 15 fields
    String fields[15];
    int commaIndex;
    int startIndex = 0;

    for(int i=0; i<15; i++) {
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

    // Check fix quality > 0
    if ((fields[0] == "$GNGGA" || fields[0] == "$GPGGA") && fields[6].toInt() > 0) {
        // Parse latitude
        if (fields[2].length() >= 4) {
            float latVal = fields[2].substring(0,2).toFloat() 
                           + fields[2].substring(2).toFloat()/60.0;
            if (fields[3] == "S") latVal = -latVal;
            latitude = latVal;
        }

        // Parse longitude
        if (fields[4].length() >= 5) {
            float lonVal = fields[4].substring(0,3).toFloat()
                           + fields[4].substring(3).toFloat()/60.0;
            if (fields[5] == "W") lonVal = -lonVal;
            longitude = lonVal;
        }
    }
}

void parseRMC(const String &sentence) {
    // RMC has up to 13 fields
    String fields[13];
    int commaIndex;
    int startIndex = 0;

    for(int i=0; i<13; i++) {
        commaIndex = sentence.indexOf(',', startIndex);
        if (commaIndex == -1) {
            fields[i] = sentence.substring(startIndex);
            break;
        }
        fields[i] = sentence.substring(startIndex, commaIndex);
        startIndex = commaIndex + 1;
    }

    // If status is 'A' => valid
    if(fields[2] == "A") {
        utcTime = formatTime(fields[1]);

        // Latitude
        if(fields[3].length() >= 4) {
            float latVal = fields[3].substring(0,2).toFloat() 
                           + fields[3].substring(2).toFloat()/60.0;
            if(fields[4] == "S") latVal = -latVal;
            latitude = latVal;
        }

        // Longitude
        if(fields[5].length() >= 5) {
            float lonVal = fields[5].substring(0,3).toFloat() 
                           + fields[5].substring(3).toFloat()/60.0;
            if(fields[6] == "W") lonVal = -lonVal;
            longitude = lonVal;
        }

        // Speed
        speed_knots = fields[7].toFloat();
        speed_mph   = speed_knots * 1.151; // approximate
    }
}
