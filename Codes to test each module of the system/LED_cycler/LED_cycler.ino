// Define RGB LED pins
#define LED_RED   LEDR
#define LED_GREEN LEDG
#define LED_BLUE  LEDB


bool I2C_connection_status = false; //Cyan
bool SD_connection_status = true;  //Magenta
bool Lidar_connection_status = true;  //Yellow
bool GPS_connection_status = false;  //BLue


void setup() {
  // put your setup code here, to run once:
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  setWhite();
}

void loop() {
  // put your main code here, to run repeatedly:
  static unsigned long previousLEDMillis = 0; // 

  unsigned long currentMillis = millis();

  //time for updating LEDs
  if ((currentMillis - previousLEDMillis >= 500) ){
    previousLEDMillis = currentMillis;
    UpdateLEDs();
  }

}

void UpdateLEDs() {
  static int currentState = 0;   // Tracks the current cycle state
  static int cycleCount = 0;     // Number of colors to display this cycle (excluding white)
  static bool lastStates[4] = {false, false, false, false}; // Keep track of the previous states of the booleans
  static bool cycleInitiated = false; // Whether we've set up the cycle

  // Current boolean states
  bool states[4];
  states[0] = I2C_connection_status;    // Corresponds to Cyan
  states[1] = SD_connection_status;     // Corresponds to Magenta
  states[2] = Lidar_connection_status;  // Corresponds to Yellow
  states[3] = GPS_connection_status;    // Corresponds to Blue

  // Check if all booleans are false
  bool allFalse = true;
  for (int i = 0; i < 4; i++) {
    if (states[i]) {
      allFalse = false;
      break;
    }
  }

  // If all booleans are false, LED is constant White
  if (allFalse) {
    setWhite();
    currentState = 0;
    cycleCount = 0;
    cycleInitiated = false;
    // Update lastStates
    for (int i = 0; i < 4; i++) {
      lastStates[i] = states[i];
    }
    return;
  }

  // If we have changed the states from the last cycle or cycle not initiated,
  // reinitialize the cycle
  bool stateChanged = false;
  for (int i = 0; i < 4; i++) {
    if (states[i] != lastStates[i]) {
      stateChanged = true;
      break;
    }
  }
  if (!cycleInitiated || stateChanged) {
    // Count how many booleans are true
    cycleCount = 0;
    for (int i = 0; i < 4; i++) {
      if (states[i]) {
        cycleCount++;
      }
    }
    currentState = 0;
    cycleInitiated = true;
  }

  // If we have a cycle to show:
  // The cycle includes showing each true boolean's color in sequence, 
  // and then white, making total cycle length = cycleCount + 1.
  int cycleLength = cycleCount + 1;

  // Determine the current output color:
  if (currentState < cycleCount) {
    // We are showing a color corresponding to one of the true booleans
    int trueIndex = 0;    // Index to identify the nth true boolean
    int varIndex = 0;     // Index of the boolean whose color we show
    for (int i = 0; i < 4; i++) {
      if (states[i]) {
        if (trueIndex == currentState) {
          varIndex = i;
          break;
        }
        trueIndex++;
      }
    }
    // Now set LED color for varIndex
    switch (varIndex) {
      case 0: setCyan(); break;     // I2C_connection_status
      case 1: setMagenta(); break;  // SD_connection_status
      case 2: setYellow(); break;   // Lidar_connection_status
      case 3: setBlue(); break;     // GPS_connection_status
    }
  } else {
    // The last step in the cycle is to show White
    setWhite();
  }

  // Move to the next state in the next cycle
  currentState++;
  if (currentState >= cycleLength) {
    currentState = 0;
  }

  // Update lastStates
  for (int i = 0; i < 4; i++) {
    lastStates[i] = states[i];
  }
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