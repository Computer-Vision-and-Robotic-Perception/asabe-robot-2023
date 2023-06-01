/*******************************************************************************
  Copyright 2016 ROBOTIS CO., LTD.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*******************************************************************************/

#include <DynamixelShield.h>
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
#define DEBUG_SERIAL soft_serial


const float DXL_PROTOCOL_VERSION = 2.0;

DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  dxl.begin(57600);
  //while(!Serial)
  
  DEBUG_SERIAL.begin(115200);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
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
  
  

}





void loop() {


     MoveForward();
     delay(65.0);

     TurnLeft();
     delay(1.0);
  int i=0 ;  
  while(i<1) {
     Stop(); }
     

}




void MoveForward () 
   {dxl.setGoalVelocity(1, -10.0, UNIT_RPM);
    dxl.setGoalVelocity(2, -10.0, UNIT_RPM);
    dxl.setGoalVelocity(3, 10.0, UNIT_RPM);
    dxl.setGoalVelocity(4, 10.0, UNIT_RPM);}
void MoveReverse()
    {dxl.setGoalVelocity(1,10.0, UNIT_RPM);
    dxl.setGoalVelocity(2, 10.0, UNIT_RPM);
    dxl.setGoalVelocity(3, -10.0, UNIT_RPM);
    dxl.setGoalVelocity(4, -10.0, UNIT_RPM); }
void TurnRight() {
     dxl.setGoalVelocity(1, 0.0, UNIT_RPM);
     dxl.setGoalVelocity(2, 0.0, UNIT_RPM);
     dxl.setGoalVelocity(3, 10.0, UNIT_RPM);
     dxl.setGoalVelocity(4, 10.0, UNIT_RPM);
      }
void TurnLeft(
  ) 
    {dxl.setGoalVelocity(1, -10.0, UNIT_RPM);
    dxl.setGoalVelocity(2, -10.0, UNIT_RPM);
    dxl.setGoalVelocity(3, 0.0, UNIT_RPM);
    dxl.setGoalVelocity(4, 0.0, UNIT_RPM);}
void Stop () {
    dxl.setGoalVelocity(1, 0.0, UNIT_RPM);
    dxl.setGoalVelocity(2, 0.0, UNIT_RPM);
    dxl.setGoalVelocity(3, 0.0, UNIT_RPM);
    dxl.setGoalVelocity(4, 0.0, UNIT_RPM); }
