//#include <HardwareSerial.h>
#include <AccelStepper.h>
#include <Servo.h>

HardwareSerial& serial_control = Serial1;
HardwareSerial& serial_report = Serial1;
HardwareSerial& serial_debug = Serial;

// The Stepper0 pins (Base Rotation)
#define STEPPER0_DIR_PIN 5
#define STEPPER0_STP_PIN 2
#define STEPPER0_END_PIN 22
// The Stepper1 pins (First Linear Actuator)
#define STEPPER1_DIR_PIN 6
#define STEPPER1_STP_PIN 3
#define STEPPER1_END_PIN 23
// The Stepper2 pins (End Rotation)
#define STEPPER2_DIR_PIN 7  
#define STEPPER2_STP_PIN 4
#define STEPPER2_END_PIN 24
// The Stepper3 pins (Second Linear Actuator)
#define STEPPER3_DIR_PIN 13  
#define STEPPER3_STP_PIN 12
#define STEPPER3_END_PIN 25
// The General Enable PIN
#define STEPPERS_EN 8
// The Servo0 pin
#define SERVO0_PIN 10
// The Servo1 pin
#define SERVO1_PIN 11

// Define the steppers and the pins the manipulator will use
AccelStepper stepper0(AccelStepper::DRIVER, STEPPER0_STP_PIN, STEPPER0_DIR_PIN);
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STP_PIN, STEPPER1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEPPER2_STP_PIN, STEPPER2_DIR_PIN);
AccelStepper stepper3(AccelStepper::DRIVER, STEPPER3_STP_PIN, STEPPER3_DIR_PIN);
Servo servo0, servo1;

// Global variables
long m = 0; // millis
bool home0 = 0;
bool home1 = 0; 
bool home2 = 0; 
bool home3 = 0;

String message = String("");
float ref[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float pos[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float vel[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// stepper ratios: 400/pi[steps/rad], 24[steps/m], 960/pi[steps/rad], 24[steps/m]
const float ratios[6] = {98.0, -24000.0, -305.6, -24000.0}; 

void home()
{
  stepper3.setSpeed(-0.05 * ratios[3]);
  while(digitalRead(STEPPER3_END_PIN) == HIGH){
    stepper3.runSpeed();
  }

  stepper2.setSpeed(-0.2 * ratios[2]);
  while(digitalRead(STEPPER2_END_PIN) == HIGH){
    stepper2.runSpeed();
  }

  stepper1.setSpeed(-0.05 * ratios[1]);
  while(digitalRead(STEPPER1_END_PIN) == HIGH){
    stepper1.runSpeed();
  }

  stepper0.setSpeed(-0.2 * ratios[0]);
  while(digitalRead(STEPPER0_END_PIN) == HIGH){
    stepper0.runSpeed();
  }
}

void measure()
{
  if(digitalRead(STEPPER0_END_PIN) == LOW){
    if(home0 == 0){
        home0 = 1;
        stepper0.setCurrentPosition(0.0);
        serial_debug.println("Stepper 0 got home");
    }  
  }
  else{home0 = 0;}

  if(digitalRead(STEPPER1_END_PIN) == LOW){
    if(home1 == 0){
        home1 = 1;
        stepper1.setCurrentPosition(0.0);
        serial_debug.println("Stepper 1 got home");
      }  
    }
    else{home1 = 0;}

  if(digitalRead(STEPPER2_END_PIN) == LOW){
    if(home2 == 0){
        home2 = 1;
        stepper2.setCurrentPosition(0.0);
        serial_debug.println("Stepper 2 got home");
      }  
    }
    else{home2 = 0;}

  if(digitalRead(STEPPER3_END_PIN) == LOW){
    if(home3 == 0){
        home3 = 1;
        stepper3.setCurrentPosition(0.0);
        serial_debug.println("Stepper 3 got home");
      }  
    }
    else{home3 = 0;}

  pos[0] = stepper0.currentPosition() / ratios[0];
  pos[1] = stepper1.currentPosition() / ratios[1];
  pos[2] = stepper2.currentPosition() / ratios[2];
  pos[3] = stepper3.currentPosition() / ratios[3];
  pos[4] = ref[4];
  pos[5] = ref[5];
  vel[0] = stepper0.speed() / ratios[0];
  vel[1] = stepper1.speed() / ratios[1];
  vel[2] = stepper2.speed() / ratios[2];
  vel[3] = stepper2.speed() / ratios[3];
  vel[4] = 0.0;
  vel[5] = 0.0;
}

void echo()
{
  if(serial_debug.available())
  {
    char in = serial_debug.read();
    
    if(in == '\n')
    {
      serial_debug.println(message);
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
  if(serial_control.available())
  {
    char in = serial_control.read();
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
           {ind2 = message.length();}
         ref[i] = message.substring(ind1, ind2).toFloat();
         if(ref[i] < 0.0)
           {ref[i] = 0.0;}
         ind1 = ind2 + 1;
      }
      serial_debug.print("Received command: ");
      serial_debug.println(message);
      message = String("");    
    }
  }
  stepper0.moveTo(ref[0] * ratios[0]);
  stepper1.moveTo(ref[1] * ratios[1]);
  stepper2.moveTo(ref[2] * ratios[2]);
  stepper3.moveTo(ref[3] * ratios[3]);
  servo0.writeMicroseconds(ref[4] + 1500); // -750, 750
  servo1.writeMicroseconds(ref[5] + 1500); // -750, 750  
}

void report(bool positions=1, bool setpoint=1, bool velocity=1)
{
  if (millis() - m > 100)
  {
    m = millis();
    if(positions)
    {
      for(byte i=0; i<6; i++)
      {
        if(i>0) serial_report.write(',');
        serial_report.print("P" + String(i) + ":");
        serial_report.print(pos[i], 3);
      }
    }
    if(setpoint)
    {
      for(byte i=0; i<6; i++)
      {
        if(i>0 or positions) serial_report.write(',');
        serial_report.print("R" + String(i) + ":");
        serial_report.print(ref[i], 3);
      }
    }
    if(velocity)
    {
      for(byte i=0; i<6; i++)
      {
        if(i>0 or positions or setpoint) serial_report.write(',');
        serial_report.print("V" + String(i) + ":");
        serial_report.print(vel[i], 3);
      }
    }
    serial_report.write('\n');
  //ref[0] = 700.0*sin(millis()/1000.0);
  //ref[1] = 700.0*sin(millis()/1000.0);
  //ref[2] = 700.0*sin(millis()/1000.0);  
  }
}

void setup()
{  
    Serial.begin(57600);
    Serial1.begin(57600);
    Serial1.begin(57600);
    
    pinMode(STEPPERS_EN, OUTPUT);
    digitalWrite(STEPPERS_EN, LOW);

    pinMode(STEPPER0_END_PIN, INPUT_PULLUP);
    pinMode(STEPPER1_END_PIN, INPUT_PULLUP);
    pinMode(STEPPER2_END_PIN, INPUT_PULLUP);
    pinMode(STEPPER3_END_PIN, INPUT_PULLUP);
    
    stepper0.setMaxSpeed(0.5 * ratios[0]);      //[rad/s]
    stepper0.setAcceleration(0.5 * ratios[0]);
    stepper1.setMaxSpeed(0.03 * ratios[1]);      //[m/s]
    stepper1.setAcceleration(0.03 * ratios[1]);
    stepper2.setMaxSpeed(0.5 * ratios[2]);      //[rad/s]
    stepper2.setAcceleration(0.5 * ratios[2]);
    stepper3.setMaxSpeed(0.03 * ratios[3]);      //[m/s]
    stepper3.setAcceleration(0.03 * ratios[3]);
    servo0.attach(SERVO0_PIN);
    servo1.attach(SERVO1_PIN); 
    serial_debug.println("Going home");
    home();
}

void loop()
{
    stepper0.run();
    stepper1.run();
    stepper2.run();
    stepper3.run();
    measure();
    update_setpoint();
    echo();
    report(1,0,1);      //(positions, setpoints, velocity);
}
