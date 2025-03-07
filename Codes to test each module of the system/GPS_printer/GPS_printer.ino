#include "Serial.h"
UART UART_GPS = UART(PC_6,PC_7);  //Using UART6 PINS

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  UART_GPS.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (UART_GPS.available()){
    while (UART_GPS.available()) {
      Serial.print(char(UART_GPS.read()));
      }
    //Serial.println();
  }
}
