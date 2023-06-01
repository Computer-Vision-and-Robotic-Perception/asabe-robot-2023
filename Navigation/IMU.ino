// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include "Wire.h"
#include "MadgwickAHRS.h"

// Initialize Madgwick Filter + other useful values
Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

uint8_t gyroRange, accelRange;

int16_t gx_int, gy_int, gz_int;
float gx, gy, gz;
float roll, pitch, yaw;
float prev_roll, prev_pitch, prev_yaw;

int16_t ax_int, ay_int, az_int;
float ax, ay, az;
float ux = 0.0f;
float uy = 0.0f;
float uz = 0.0f;
float x = 0.0f;
float y = 0.0f;
float z = 0.0f;
float prev_x, prev_y, prev_z;

float nominalState[3] = {0.0f, 0.0f, 0.0f};
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
/*
 * gyroRange = 0, 1, 2, or 3 (default 0)
 * gyroRange | Full Scale Range   | LSB Sensitivity
 * ----------+--------------------+----------------
 * 0         | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1         | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2         | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3         | +/- 2000 degrees/s | 16.4 LSB/deg/sc
*/
    gyroRange = 0; 

/*
 * accelRange = 0, 1, 2, or 3 (default 0)
 * accelRange | Full Scale Range | LSB Sensitivity
 * -----------+------------------+----------------
 * 0          | +/- 2g           | 8192 LSB/mg
 * 1          | +/- 4g           | 4096 LSB/mg
 * 2          | +/- 8g           | 2048 LSB/mg
 * 3          | +/- 16g          | 1024 LSB/mg
 */
    accelRange = 0;
    
    initializeIMU(gyroRange, accelRange);    
    initializeMadgwick();
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) {
      Serial.print("Failed...");
      return;
    }

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        mpuIntStatus = mpu.getIntStatus();
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        float startTime = micros();

        // Save previous measurement
        prev_x = x;
        prev_y = y;
        prev_z = z;
        
        // get measurements + convert values
        getAccelMeasurement();
        getAngVel();
        convertReading(gyroRange, accelRange, gx, gy, gz, ax, ay, az);     

        // update the filter, which computes orientation
        filter.updateIMU(gx, gy, gz, ax, ay, az); // degrees -> radians
        
        // print the heading, pitch and roll
        roll = filter.getRoll();
        pitch = filter.getPitch();
        yaw = filter.getYaw();
        float elapsedTime = (micros() - startTime) / (1000000.0f);
        getXVel(ux, elapsedTime); // Write this function
        getYVel(uy, elapsedTime);
        getZVel(uz, elapsedTime);

        getX(x, elapsedTime); // Write this function
        getY(y, elapsedTime);
        getZ(z, elapsedTime);

        calcNominalState();
   /*
        Serial.print("Orientation: ");
        Serial.print(yaw);
        Serial.print(" ");
        Serial.print(pitch);
        Serial.print(" ");
        Serial.println(roll); 
  //  */
   /*
        Serial.print("Position: ");
        Serial.print(x);
        Serial.print(" ");
        Serial.print(y);
        Serial.print(" ");
        Serial.print(elapsedTime);
        Serial.print(" ");
        Serial.println(z); 
  //  */

        Serial.print("Nominal State: ");
        Serial.print(nominalState[0]);
        Serial.print(" ");
        Serial.print(nominalState[1]);
        Serial.print(" ");
        Serial.print(nominalState[2]);
        Serial.println(" ");
    }
    
}

void getAngVel() {
  gx_int = mpu.getRotationX();
  gy_int = mpu.getRotationY();
  gz_int = mpu.getRotationZ();
}

/*
 *   getAccekMeasuremen
 *   INPUT: N/A
 *   OUTPUT: gets raw accel value from IMU
 *   
 *   aaReal.x = Accel x
 *   aaReal.y = Accel y
 *   aaReal.z = Accel z
 */

void getAccelMeasurement() {
  ax_int = mpu.getAccelerationX();
  ay_int = mpu.getAccelerationY();
  az_int = mpu.getAccelerationZ();
}

void getXVel(float& ux, float elapsedTime) {
  ux += ax*elapsedTime;
}

void getYVel(float& uy, float elapsedTime) {
  uy += ay*elapsedTime;
}

void getZVel(float& uz, float elapsedTime) {
  uz += az*elapsedTime;
}

void getX(float& x, float elapsedTime) {
  x += ux * elapsedTime + (0.5f * ax * elapsedTime * elapsedTime);
}

void getY(float& y, float elapsedTime) {
  y += uy * elapsedTime + (0.5f * ay * elapsedTime * elapsedTime);
}

void getZ(float& z, float elapsedTime) {
  z += uz * elapsedTime + (0.5f * az * elapsedTime * elapsedTime);
}

void calcNominalState() {
  float deltaX = x - prev_x;
  float deltaY = y - prev_y;
  float rho = sqrt(deltaX * deltaX + deltaY* deltaY);

  //nominalState[0] = {x + rho*cos(yaw), y + rho*sin(yaw), yaw};
  nominalState[0] = x + rho*cos(yaw);
  nominalState[1] = y + rho*sin(yaw);
  nominalState[2] = yaw;
}


void convertReading(uint8_t gyroRange, uint8_t accelRange, float& gx, float& gy, float& gz, float& ax, float& ay, float& az) {
  // convert reading to degrees/s
  float gyroMax;
  if (gyroRange == 3){
    gyroMax = 2000.0;
  }
  else if (gyroRange == 2){
    gyroMax = 1000.0;
  }
  else if (gyroRange == 1){
    gyroMax = 500.0;
  }
  else {
    gyroMax = 250.0;
  }
  gx = gx_int * (gyroMax/32767.0);
  gy = gy_int * (gyroMax/32767.0);
  gz = gz_int * (gyroMax/32767.0);

  // convert reading to m/s
  float accelMax;
  if (accelRange == 3){
    accelMax = 16.0 * 9.81;
  }
  else if (accelRange == 2){
    accelMax = 8.0 * 9.81;
  }
  else if (accelRange == 1){
    accelMax = 4.0 * 9.81;
  }
  else {
    accelMax = 2.0f * 9.81f;
  }
  ax = ax_int * (accelMax/32767.0);
  ay = ay_int * (accelMax/32767.0);
  az = az_int * (accelMax/32767.0);
}

void initializeIMU(uint8_t gyroRange, uint8_t accelRange) {
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    
    // Set gyro range setting
    mpu.setFullScaleGyroRange(gyroRange);
    Serial.print("Gyro Range setting selection: ");
    Serial.println(mpu.getFullScaleGyroRange());

    // Set accel range setting
    mpu.setFullScaleAccelRange(accelRange);
    Serial.print("Accel Range setting selection: ");
    Serial.println(mpu.getFullScaleAccelRange());
}

void initializeMadgwick(){
  // initialize variables to pace updates to correct rate
    microsPerReading = 1000000 / 25;
    microsPrevious = micros();
    Serial.print("Madgwick initialized...");
}
