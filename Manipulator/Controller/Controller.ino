#include <AccelStepper.h>
#include <Servo.h>

// The Stepper0 pins (Base Rotation)
#define STEPPER0_DIR_PIN 8
#define STEPPER0_STP_PIN 9
// The Stepper1 pins (First Linear Actuator)
#define STEPPER1_DIR_PIN 2
#define STEPPER1_STP_PIN 3
// The Stepper2 pins (End Rotation)
#define STEPPER2_DIR_PIN 6  
#define STEPPER2_STP_PIN 7
// The Stepper3 pins (Second Linear Actuator)
#define STEPPER3_DIR_PIN 4  
#define STEPPER3_STP_PIN 5
// The Servo0 pin
#define SERVO0_PIN 10
// The Servo1 pin
#define SERVO1_PIN 11

// Define the steppers and the pins the will use
AccelStepper stepper0(AccelStepper::DRIVER, STEPPER0_STP_PIN, STEPPER0_DIR_PIN);
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STP_PIN, STEPPER1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEPPER2_STP_PIN, STEPPER2_DIR_PIN);
AccelStepper stepper3(AccelStepper::DRIVER, STEPPER3_STP_PIN, STEPPER3_DIR_PIN);
Servo servo0, servo1;

// Global variables
long m = 0;
String message = String("");
float ref[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float pos[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float vel[6] = {1.0, 1.0, 1.0, 1.0, 1.0, 0.0};

void measure()
{
  pos[0] = stepper0.currentPosition();
  pos[1] = stepper1.currentPosition();
  pos[2] = stepper2.currentPosition();
  pos[3] = stepper3.currentPosition();
  pos[4] = ref[4];
  pos[5] = ref[5];
  vel[0] = stepper0.speed();
  vel[1] = stepper1.speed();
  vel[2] = stepper2.speed();
  vel[3] = stepper2.speed();
  vel[4] = 0.0;
  vel[5] = 0.0;
}

void echo()
{
  if(Serial.available())
  {
    char in = Serial.read();
    
    if(in == '\n')
    {
      Serial.println(message);
      message = String("");    
    }
    else
    {
      message.concat(in);
    }
  }
}

void update_setpoint()
{
  if(Serial.available())
  {
    char in = Serial.read();
    if(in >= '0' and in <= '9' or in=='-' or in==',' or in=='.')
    {
      message.concat(in);
    }
    else if(in == '\n')
    {
      int ind1 = 0;
      int ind2 = 0;
      for(byte i=0; i<6; i++)
      {
         ind2 = message.indexOf(',', ind1);
         if(ind2 == -1)
           ind2 = message.length();
         ref[i] = message.substring(ind1, ind2).toFloat();
         ind1 = ind2 + 1;
      }
      Serial.println(message);
      message = String("");    
    }
  }  
}

void report(bool positions=1, bool setpoint=1, bool velocity=1)
{
  if (millis() - m > 50)
  {
    m = millis();
    if(positions)
    {
      for(byte i=0; i<6; i++)
      {
        if(i>0) Serial.print(',');
        Serial.print("P" + String(i) + ":");
        Serial.print(pos[i]);
      }
    }
    if(setpoint)
    {
      for(byte i=0; i<6; i++)
      {
        if(i>0 or positions) Serial.print(',');
        Serial.print("R" + String(i) + ":");
        Serial.print(ref[i]);
      }
    }
    if(velocity)
    {
      for(byte i=0; i<6; i++)
      {
        if(i>0 or positions or setpoint) Serial.print(',');
        Serial.print("V" + String(i) + ":");
        Serial.print(vel[i]);
      }
    }
    Serial.write('\n');
  //ref[0] = 700.0*sin(millis()/1000.0);
  //ref[1] = 700.0*sin(millis()/1000.0);
  //ref[2] = 700.0*sin(millis()/1000.0);  
  }
}

void setup()
{  
    Serial.begin(2e6);
    stepper0.setMaxSpeed(800.0);
    stepper0.setAcceleration(1600.0);
    stepper1.setMaxSpeed(800.0);
    stepper1.setAcceleration(1600.0);
    stepper2.setMaxSpeed(800.0);
    stepper2.setAcceleration(1600.0);
    stepper3.setMaxSpeed(800.0);
    stepper3.setAcceleration(1600.0);
    servo0.attach(SERVO0_PIN);
    servo1.attach(SERVO1_PIN);   
}

void loop()
{
    stepper0.moveTo(ref[0]);
    stepper1.moveTo(ref[1]);
    stepper2.moveTo(ref[2]);
    stepper3.moveTo(ref[3]);
    servo0.writeMicroseconds(ref[4] + 1500); // -750, 750
    servo1.writeMicroseconds(ref[5] + 1500); // -750, 750
    stepper0.run();
    stepper1.run();
    stepper2.run();
    stepper3.run();
    measure();
    update_setpoint();
    //echo();
    //report(1,1,0);      //(positions, setpoints, velocity);
}