#include <DynamixelShield.h>
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(10, 11); // DYNAMIXELShield UART RX/TX
#define DEBUG_SERIAL soft_serial

const float DXL_PROTOCOL_VERSION = 2.0;
String message = String("");
const String names[3] = {"straight", "rotate", "circle"};
DynamixelShield dxl;
byte id;
byte mode = 0;
byte pin_mode = 51;

float ref[4] = {120.0, 120.0, 120.0, 120.0};


void setup() {
  pinMode(pin_mode, INPUT_PULLUP);
  DEBUG_SERIAL.begin(57600);
  dxl.begin(57600);
  DEBUG_SERIAL.println("Debug started.");
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  DEBUG_SERIAL.println("Ping...");
  dxl.ping(1);
  dxl.ping(2);
  dxl.ping(3);
  dxl.ping(4);
  dxl.torqueOff(1);
  dxl.torqueOff(2);
  dxl.torqueOff(3);
  dxl.torqueOff(4);
  dxl.setOperatingMode(1, OP_VELOCITY);
  dxl.setOperatingMode(2, OP_VELOCITY);
  dxl.setOperatingMode(3, OP_VELOCITY);
  dxl.setOperatingMode(4, OP_VELOCITY);
  dxl.torqueOn(1);
  dxl.torqueOn(2);
  dxl.torqueOn(3);
  dxl.torqueOn(4);
  dxl.setGoalVelocity(1, ref[0]);
  dxl.setGoalVelocity(2, ref[1]);
  dxl.setGoalVelocity(3, ref[2]);
  dxl.setGoalVelocity(4, ref[3]);
}

void loop() {
  update_ref();
  if (digitalRead(pin_mode) == LOW){
    mode++;
    DEBUG_SERIAL.println(names[mode % 3]);
    delay(500);
    }
  switch(mode % 3){
    case 0:
      move_forward();
      break;
    case 1:
      rotate();
      break;
    case 2:
      circle();
      break; 
  }
}

void update_ref()
{
  if(DEBUG_SERIAL.available())
  {
    char in = DEBUG_SERIAL.read();
    if(in >= '0' and in <= '9' or in=='-' or in==',' or in=='.')
    {
      message.concat(in);
    }
    else if(in == '\n')
    {
      int ind1 = 0;
      int ind2 = 0;
      for(byte i=0; i<4; i++)
      {
         ind2 = message.indexOf(',', ind1);
         if(ind2 == -1)
           ind2 = message.length();
         ref[i] = message.substring(ind1, ind2).toFloat();
         ind1 = ind2 + 1;
      }
      DEBUG_SERIAL.println(message);
      message = String("");  
    }
  }  
}

void move_forward(){
  dxl.setGoalVelocity(1, ref[0]);
  dxl.setGoalVelocity(2, ref[0]);
  dxl.setGoalVelocity(3, -ref[0]);
  dxl.setGoalVelocity(4, -ref[0]);  
}

void rotate(){
  dxl.setGoalVelocity(1, ref[0]);
  dxl.setGoalVelocity(2, ref[0]);
  dxl.setGoalVelocity(3, ref[0]);
  dxl.setGoalVelocity(4, ref[0]); 
}

void circle(){
  dxl.setGoalVelocity(1, ref[0]);
  dxl.setGoalVelocity(2, ref[0]);
  dxl.setGoalVelocity(3, -ref[0]/4);
  dxl.setGoalVelocity(4, -ref[0]/4); 
}
