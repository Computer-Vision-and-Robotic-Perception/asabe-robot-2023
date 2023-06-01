#include "PreMo.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include "konfig.h"
#include "matrix.h"
#include <elapsedMillis.h>
#include "ekf.h"
#include <DynamixelShield.h>
#include <SoftwareSerial.h>
#include "PreMo.h"
#include <RPLidar.h>
#define RPLIDAR_MOTOR 22

/** ---------------------------------------------------------------------------------------------------------------------------
    All the variables are below
  ---------------------------------------------------------------------------------------------------------------------------**/

const double KP = 50;
const double KD = 1;
// PID for motors (for twist method)
const double KP_motor = 1.5;
const double KI_motor = 0;
#define P_INIT      (10.)
#define Q_INIT      (1e-6)
#define R_INIT_ACC  (0.0015/10.)
const double RADIUS = 10;
const double LENGTH = 50;
double x_pos;
double y_pos;
double theta_pos;
double Vl = 0; //Left velocity
double Vr = 0; //Right velocity
const int PATH_FOLLOW_SPEED = 100; //In percentage
const int MOTOR_SPEED_MAX = 10; //In rpm
float EKF_PINIT_data[SS_X_LEN * SS_X_LEN] = {P_INIT,      0,      0,
                                             0,      P_INIT, 0,
                                             0,      0,      P_INIT,
                                            };

Matrix EKF_PINIT(SS_X_LEN, SS_X_LEN, EKF_PINIT_data);
float_prec EKF_QINIT_data[SS_X_LEN * SS_X_LEN] = {Q_INIT, 0,      0,
                                                  0,      Q_INIT, 0,
                                                  0,      0,      Q_INIT
                                                 };
Matrix EKF_QINIT(SS_X_LEN, SS_X_LEN, EKF_QINIT_data);

float_prec EKF_RINIT_data[SS_Z_LEN * SS_Z_LEN] = {R_INIT_ACC, 0,          0,
                                                  0,          R_INIT_ACC, 0,
                                                  0,          0,          R_INIT_ACC
                                                 };
Matrix EKF_RINIT(SS_Z_LEN, SS_Z_LEN, EKF_RINIT_data);
/* Nonlinear & linearization function ------------------------------------------------------------------------------- */
bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U);
bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U);
bool Main_bCalcJacobianF(Matrix& F, const Matrix& X, const Matrix& U);
bool Main_bCalcJacobianH(Matrix& H, const Matrix& X, const Matrix& U);

Matrix Position(SS_X_LEN, 1);
Matrix Y(SS_Z_LEN, 1);
Matrix U(SS_U_LEN, 1);
SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
#define DEBUG_SERIAL soft_serial
const float DXL_PROTOCOL_VERSION = 2.0;
bool         is_initialised=false;

DynamixelShield dxl;
RPLidar lidar;

void leftForward (float SPEED)
{ dxl.setGoalVelocity(1, -SPEED, UNIT_RPM);
  dxl.setGoalVelocity(2, -SPEED, UNIT_RPM);
}

void rightForward(float SPEED) {
  dxl.setGoalVelocity(3, SPEED, UNIT_RPM);
  dxl.setGoalVelocity(4, SPEED, UNIT_RPM);
}
void leftReverse(float SPEED) {
  dxl.setGoalVelocity(1, SPEED, UNIT_RPM);
  dxl.setGoalVelocity(2, SPEED, UNIT_RPM);
}
void rightReverse(float SPEED) {
  dxl.setGoalVelocity(3, -SPEED, UNIT_RPM);
  dxl.setGoalVelocity(4, -SPEED, UNIT_RPM);
}
void stopMotors () {
  dxl.setGoalVelocity(1, 0.0, UNIT_RPM);
  dxl.setGoalVelocity(2, 0.0, UNIT_RPM);
  dxl.setGoalVelocity(3, 0.0, UNIT_RPM);
  dxl.setGoalVelocity(4, 0.0, UNIT_RPM);
}



EKF EKF_Navigation(Position, EKF_PINIT, EKF_QINIT, EKF_RINIT,
                   Main_bUpdateNonlinearX, Main_bUpdateNonlinearY, Main_bCalcJacobianF, Main_bCalcJacobianH);
MotorManager motorManager(leftForward, leftReverse, rightForward, rightReverse, stopMotors);
PreMo premo(RADIUS, LENGTH, KP, KD, KP_motor, KI_motor, &motorManager, &x_pos , &y_pos, &theta_pos, &Vl, &Vr);




/** --------------------------------------------------
        SETUP
  ----------------------------------------------------**/

void setup() {
  Serial.begin(115200);
  EKF_Navigation.vReset(Position, EKF_PINIT, EKF_QINIT, EKF_RINIT);
  setupPreMoStuff();
  setupMotorStuff();
  setupLIDAR();
}

/**----------------------------------------------------------------------
                     VARIABLES
  ----------------------------------------------------------------------**/

elapsedMillis timerLed, timerEKF;
uint64_t u64compuTime;
float delta_D = 0;
float delta_phi = 0;
float dt = 0.01;
float states_y[5] = {1.0, 1.2, 1.3, 1.4, 1.4};
float states_x[5] = {1.0, 1.0, 1.0, 1.0, 1.0};
float states_t[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float u[5] = {0.1, 0.1, 0.1, 0.1, 0.1};
float t[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
double Distances_lid[360];
double Angles_lid[360];



/**----------------------------------------------------------------------
                     MAIN LOOP
  ----------------------------------------------------------------------**/

void loop() {
  int a = 0;
  if (is_initialised==false)  
     {lidar.startScan();
     while (a < 180) {
        if (IS_OK(lidar.waitPoint()) && lidar.getCurrentPoint().angle<180) {
          Distances_lid[a] = lidar.getCurrentPoint().distance;
          Angles_lid[a] = lidar.getCurrentPoint().angle;
          a++; } }

     //Get initial x and y from it;
       is_initialised=true; }

  
  else {  

  if (timerEKF >= SS_DT_MILIS) {
    timerEKF = 0;
    /**-----------------------------------
                       LIDAR
      -----------------------------------**/
    float x_pos; //from LiDAR
    float y_pos; //from LiDAR
    float theta_pos; //from LIDAR

    /**-----------------------------------
                      MOTOR
      -----------------------------------**/

    float Vl = dxl.getPresentVelocity(1, UNIT_RPM);
    float Vr = dxl.getPresentVelocity(3, UNIT_RPM);
    GetControlInputs(Vl, Vr, dt, delta_D, delta_phi);

    /**-----------------------------------
                      KALMAN FILTER
      -----------------------------------**/

    U[0][0] = delta_D;  U[1][0] = delta_phi;
    Y[0][0] = x_pos; Y[1][0] = y_pos; Y[2][0] = theta_pos;
    EKF_Navigation.bUpdate(Y, U);
    Position = EKF_Navigation.GetX();

    /**-----------------------------------
                       Tracking
      -----------------------------------**/
    premo.loop();
  }

}
}

/** --------------------------------------------------------------------------------
    All the functions used are below:
  ---------------------------------------------------------------------------------- **/

/** -------------------------------------
              TRACKING FUNCTIONS
  ----------------------------------------**/


void setupPreMoStuff() {
  motorManager.setSpeedLimits(0, MOTOR_SPEED_MAX);
  premo.twistBothMotors(false);
  premo.setPathFollowSpeed(PATH_FOLLOW_SPEED);
}

void pathFollowing(float pathX[], float pathY[], float pathLength) {
  delay(1000);
  premo.startPathFollowing(pathX, pathY, pathLength);
}

/** -------------------------------------
              MOTOR SETUP
  ----------------------------------------**/


void setupMotorStuff() {
  dxl.begin(57600);
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

/** -------------------------------------
              LIDAR SETUP
  ----------------------------------------**/
  void setupLIDAR() 
  {lidar.begin(Serial3);
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  analogWrite(RPLIDAR_MOTOR, 255); }


/** -------------------------------------
             KALMAN FILTER FUNCTIONS
  ----------------------------------------**/


bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U)
{
  float x_pos, y_pos, theta_pos, delta_D, delta_phi;
  x_pos = X[0][0];
  y_pos = X[1][0];
  theta_pos = X[2][0];

  delta_D = U[0][0];
  delta_phi = U[1][0];

  X_Next[0][0] = x_pos + (delta_D * cos(theta_pos + delta_phi));
  X_Next[1][0] = x_pos + (delta_D * sin(theta_pos + delta_phi));
  X_Next[2][0] = theta_pos + delta_phi;

  return true;
}

bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U)
{

  float x_pos, y_pos, theta_pos;
  x_pos = X[0][0];
  y_pos = X[1][0];
  theta_pos = X[2][0];

  Y[0][0] = x_pos;

  Y[1][0] = y_pos;

  Y[2][0] = theta_pos;

  return true;
}

bool Main_bCalcJacobianF(Matrix& F, const Matrix& X, const Matrix& U)
{
  float x_pos, y_pos, theta_pos, delta_D, delta_phi;
  delta_D = U[0][0];
  delta_phi = U[1][0];
  x_pos = X[0][0];
  y_pos = X[1][0];
  theta_pos = X[2][0];


  F[0][0] =  1.0;
  F[1][0] =  0.0;
  F[2][0] =  0.0;


  F[0][1] =  0.0;
  F[1][1] =  1.0;
  F[2][1] =  0.0;


  F[0][2] =  -delta_D * sin(theta_pos + delta_phi);
  F[1][2] =  delta_D * cos(theta_pos + delta_phi);
  F[2][2] =  1.000;


  return true;
}

bool Main_bCalcJacobianH(Matrix& H, const Matrix& X, const Matrix& U)
{
  H[0][0] = 1.0;
  H[1][0] = 0.0;
  H[2][0] = 0.0;


  H[0][1] = 0.0;
  H[1][1] = 1.0;
  H[2][1] = 0.0;

  H[0][2] = 0.0;
  H[1][2] = 0.0;
  H[2][2] = 1.0;

  return true;
}


void GetControlInputs(float &Vl, float &Vr, float dt, float &delta_D, float &delta_phi)
{ float pi = 3.141592;
  int   radius = 10;
  int   distance = 20; //between the two wheels
  auto linear_Vr = ((Vr * 2 * pi) / 60) * radius;
  auto linear_Vl = ((Vl * 2 * pi) / 60) * radius;
  delta_D = ((linear_Vr + linear_Vl) / 2) * dt;
  delta_phi = ((linear_Vr - linear_Vl) / distance) * dt;
}

void SPEW_THE_ERROR(char const * str)
{
#if (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_PC)
  cout << (str) << endl;
#elif (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_EMBEDDED_ARDUINO)
  Serial.println(str);
#else
  /* Silent function */
#endif
  while (1);
}
