// Define RGB LED pins
#define LED_RED   LEDR
#define LED_GREEN LEDG
#define LED_BLUE  LEDB

// Setup function
void setup() {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
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

// Main loop function
void loop() {
  // Example usage of color functions
  setRed();
  delay(500);

  setGreen();
  delay(500);

  setBlue();
  delay(500);

  setYellow();
  delay(500);

  setMagenta();
  delay(500);

  setCyan();
  delay(500);

  setWhite();
  delay(500);

  turnOffLED();  // Turn off all LEDs (Black)
  delay(500);
}
