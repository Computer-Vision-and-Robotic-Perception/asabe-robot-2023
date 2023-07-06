#include <Dynamixel2Arduino.h>
#include <HardwareSerial.h>

DYNAMIXEL::SerialPortHandler dynamixel_serial(Serial, 2);
HardwareSerial& mirror_serial = Serial1;

void setup() {
  dynamixel_serial.begin();
  mirror_serial.begin(57600);
}

void loop() {
  if(mirror_serial.available()){
    dynamixel_serial.write(mirror_serial.read());
  }
  if(dynamixel_serial.available()){
    mirror_serial.write(dynamixel_serial.read());
  }
}
