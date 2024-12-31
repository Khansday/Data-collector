#ifndef LEDCONTROL_H
#define LEDCONTROL_H

#include <Arduino.h>

// If your Portenta’s LEDs have different macros/pins, adjust here:
#define LED_RED   LEDR
#define LED_GREEN LEDG
#define LED_BLUE  LEDB

// LED control functions
void initLEDs();
void updateLEDs(bool i2cOK, bool sdOK, bool lidarOK, bool gpsOK);
void blinkAllLEDs();

// Individual color setters
void turnOffLED();
void setRed();
void setGreen();
void setBlue();
void setCyan();
void setMagenta();
void setYellow();
void setWhite();

#endif
