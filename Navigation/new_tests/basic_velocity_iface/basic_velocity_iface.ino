#include <DynamixelShield.h>
#include <SoftwareSerial.h>
#include <RPLidar.h>

SoftwareSerial soft_serial(10, 11); // DYNAMIXELShield UART RX/TX
#define DEBUG_SERIAL soft_serial
#define RPLIDAR_MOTOR 3

const float DXL_PROTOCOL_VERSION = 2.0;
String message = String("");
const String names[3] = {"straight", "rotate", "circle"};

DynamixelShield dxl;
RPLidar lidar;

byte id;
byte mode = 0;
byte pin_mode = 12;

int count = 0;

float ref[4] = {120.0, 120.0, 120.0, 120.0};


void setup() {
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  pinMode(pin_mode, INPUT_PULLUP);
  DEBUG_SERIAL.begin(57600);
  lidar.begin(Serial1);
  dxl.begin(57600);
  delay(2000);
  // DEBUG_SERIAL.println("Debug started.");
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // DEBUG_SERIAL.println("Ping...");
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
  read_lidar();
  update_ref();
  if (digitalRead(pin_mode) == LOW){
    mode++;
    // DEBUG_SERIAL.println(names[mode % 3]);
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

void read_lidar()
{
  count++;
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance;   //distance value in mm unit
    float angle    = lidar.getCurrentPoint().angle;      //anglue value in degree
    bool  startBit = lidar.getCurrentPoint().startBit;   //whether this point is belong to a new scan
    byte  quality  = lidar.getCurrentPoint().quality;    //quality of the current measurement
    if(count % 1 == 0 and quality > 5 and (angle > 340 or angle < 20) and angle < 360 and distance > 150 and distance < 3500){
      //DEBUG_SERIAL.print("angle:");
      //DEBUG_SERIAL.print(angle);
      //DEBUG_SERIAL.print(",");+
      DEBUG_SERIAL.print("distance:");
      DEBUG_SERIAL.print(distance);
      //DEBUG_SERIAL.print(",");
      //DEBUG_SERIAL.print("startBit:");
      //DEBUG_SERIAL.print(startBit);
      //DEBUG_SERIAL.print(",");
      //DEBUG_SERIAL.print("quality:");
      //DEBUG_SERIAL.print(quality);
      DEBUG_SERIAL.write("\n");

      if(distance < 300){mode = 1;} // rotate
      else {mode = 0;} //straight
    }
  } else {
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
    // try to detect RPLIDAR... 
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       // DEBUG_SERIAL.println("detected...");
       lidar.startScan();
       analogWrite(RPLIDAR_MOTOR, 255); // speed 0~255
       delay(1000);
    }
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
      // DEBUG_SERIAL.println(message);
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
