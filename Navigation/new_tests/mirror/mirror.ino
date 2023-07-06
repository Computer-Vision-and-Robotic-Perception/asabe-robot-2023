#include <SoftwareSerial.h>
//SoftwareSerial soft_serial(7, 8); // For Arduino UNO
SoftwareSerial soft_serial(10, 11); // For Arduino MEGA (original valus do not work!)
#define DEBUG_SERIAL soft_serial

void setup() {
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(57600);
  Serial.begin(57600);
  //Serial.println("Starting bebugger...");
}

void loop() {
  char c;
  if(DEBUG_SERIAL.available())
  {
    c = DEBUG_SERIAL.read();
    Serial.write(c);
  }
  if(Serial.available())
  {
    c = Serial.read();
    DEBUG_SERIAL.write(c);
  }
}
