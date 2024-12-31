#include "LEDControl.h"

// Tracks the current cycle state and so forth in updateLEDs()
static int s_currentState = 0;   
static int s_cycleCount  = 0;     
static bool s_lastStates[4] = {false, false, false, false}; 
static bool s_cycleInitiated = false;

/**
 * @brief Initializes the pins for the on-board RGB LED.
 */
void initLEDs() {
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    turnOffLED();
}

/**
 * @brief Blinks all LEDs in quick sequence (for startup indicator).
 */
void blinkAllLEDs() {
    turnOffLED();
    setRed();    delay(50);
    setGreen();  delay(50);
    setBlue();   delay(50);
    turnOffLED();delay(100);
}

/**
 * @brief Update the LED color(s) based on connection statuses.
 *
 * Logic:
 *  - If all true (i.e., everything OK) -> White constant
 *  - Otherwise, cycle through each false condition with a color:
 *      - !i2cOK => Cyan
 *      - !sdOK => Magenta
 *      - !lidarOK => Yellow
 *      - !gpsOK => Blue
 *    and then a brief White.
 */
void updateLEDs(bool i2cOK, bool sdOK, bool lidarOK, bool gpsOK) {
    // Current boolean states: false means error
    bool states[4];
    states[0] = !i2cOK;     
    states[1] = !sdOK;      
    states[2] = !lidarOK;   
    states[3] = !gpsOK;     

    // Check if all are false => everything is actually OK => show White
    bool allFalse = true;
    for(int i = 0; i < 4; i++) {
        if (states[i]) { 
            allFalse = false; 
            break; 
        }
    }
    // If everything is OK, set White
    if (allFalse) {
        setWhite();
        s_currentState = 0;
        s_cycleCount   = 0;
        s_cycleInitiated = false;
        for(int i=0; i<4; i++) s_lastStates[i] = states[i];
        return;
    }

    // Check if states changed => re-init the cycle
    bool stateChanged = false;
    for(int i=0; i<4; i++) {
        if (states[i] != s_lastStates[i]) {
            stateChanged = true;
            break;
        }
    }
    if (!s_cycleInitiated || stateChanged) {
        // Count how many are true
        s_cycleCount = 0;
        for(int i=0; i<4; i++) {
            if (states[i]) {
                s_cycleCount++;
            }
        }
        s_currentState   = 0;
        s_cycleInitiated = true;
    }

    // total cycle length = s_cycleCount + 1 (the "+1" is White)
    int cycleLength = s_cycleCount + 1;

    // Determine which color to show now
    if(s_currentState < s_cycleCount) {
        // We are showing one of the error colors
        int trueIndex = 0;  
        int varIndex  = 0;
        for(int i=0; i<4; i++) {
            if(states[i]) {
                if(trueIndex == s_currentState) {
                    varIndex = i;
                    break;
                }
                trueIndex++;
            }
        }

        switch(varIndex) {
            case 0: setCyan();    break; // I2C
            case 1: setMagenta(); break; // SD
            case 2: setYellow();  break; // Lidar
            case 3: setBlue();    break; // GPS
        }
    } else {
        // Last step: show White
        setWhite();
    }

    // Next step for next time
    s_currentState++;
    if(s_currentState >= cycleLength) {
        s_currentState = 0;
    }

    // Update last states
    for(int i=0; i<4; i++) {
        s_lastStates[i] = states[i];
    }
}


// -----------------------------------------------------------
// Individual color setting functions
// -----------------------------------------------------------
void turnOffLED() {
    digitalWrite(LED_RED,   HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE,  HIGH);
}

void setRed() {
    digitalWrite(LED_RED,   LOW);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE,  HIGH);
}

void setGreen() {
    digitalWrite(LED_RED,   HIGH);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE,  HIGH);
}

void setBlue() {
    digitalWrite(LED_RED,   HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE,  LOW);
}

void setCyan() {
    digitalWrite(LED_RED,   HIGH);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE,  LOW);
}

void setMagenta() {
    digitalWrite(LED_RED,   LOW);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE,  LOW);
}

void setYellow() {
    digitalWrite(LED_RED,   LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE,  HIGH);
}

void setWhite() {
    digitalWrite(LED_RED,   LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE,  LOW);
}
