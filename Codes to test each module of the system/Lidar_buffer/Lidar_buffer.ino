#define BUFFER_SIZE 1000  // Define the size of the buffer

// Create a struct to hold angle, distance, and signal strength
struct Measurement {
  int angle;
  int distance;
  byte signalStrength;
};

Measurement buffer[BUFFER_SIZE];  // Array to store  elements
int bufferIndex = 0;              // Index to track current position in the buffer

void setup() {
  Serial.begin(250000);      // Start the hardware serial port for logging
  Serial1.begin(230400);     // Start the software serial port to communicate with the LiDAR
}

void loop() {
  
  //unsigned long int timer = micros(); //just for debug

  if (Serial1.available() > 44) {  // Ensure there's enough data for one complete packet
    byte dataPacket[44];
    for (int i = 0; i < 44; i++) {
      dataPacket[i] = Serial1.read();
    }

    // Check for the start character and packet descriptor
    if (dataPacket[0] == 0x54 && dataPacket[1] == 0x2C) {
      // Extract the radar speed
      int radarSpeed = dataPacket[2] + (dataPacket[3] << 8);
      
      // Convert start and end angles from two bytes to a single value and scale
      float startAngle = (dataPacket[4] + (dataPacket[5] << 8)) * 0.01;
      float endAngle = (dataPacket[40] + (dataPacket[41] << 8)) * 0.01;
      
      // Process each measurement point
      for (int i = 6; i < 40; i += 3) {
        int distance = dataPacket[i] + (dataPacket[i+1] << 8);
        byte signalStrength = dataPacket[i+2];
        
        // Interpolate the angle for this measurement point
        float step = (endAngle - startAngle) / 11;  // There are 12 data points
        float angle1 = startAngle + step * ((i - 6) / 3);
        int angle = angle1;

        // Add data to the buffer
        buffer[bufferIndex].angle = angle;
        buffer[bufferIndex].distance = distance;
        buffer[bufferIndex].signalStrength = signalStrength;
        bufferIndex++;
        //Serial.println(bufferIndex); just for debug
        // If buffer is full, print all the data and reset the buffer index
        if (bufferIndex >= BUFFER_SIZE) {
          for (int j = 0; j < BUFFER_SIZE; j++) {
            Serial.print(buffer[j].angle);
            Serial.print("\t\t");
            Serial.print(buffer[j].distance);
            Serial.print("\t\t");
            Serial.println(buffer[j].signalStrength);
          }
          bufferIndex = 0;  // Reset the buffer
        }
      }
    }
  }
  
  //Serial.println(micros()-timer); //just for debug 

  delay(1);
}
