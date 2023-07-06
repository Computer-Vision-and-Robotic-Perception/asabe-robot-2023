#include <stdint.h>
#include "MultiWii.hpp"

#ifndef __MSP_SEND__
#define __MSP_SEND__(NAME, CODE) \
void MSP_##NAME(const int fd, MSP_##NAME##_t * const data) { \
    MultiWiiPacket_t packet = {           \
        .size=sizeof(*data),              \
        .code=CODE,                       \
        .data=(uint8_t *)data,            \
    };                                    \
    MultiWii_send(fd, &packet);           \
}
#endif

// #define ERR_SIZE_MISMATCH 0b0001
// #define ERR_SIZE_MISMATCH 0b0010
// #define ERR_SIZE_MISMATCH 0b0100

#ifndef __MSP_RECV__
#define __MSP_RECV__(NAME, CODE) \
void MSP_##NAME(const int fd, MSP_##NAME##_t * const data) { \
    MultiWiiPacket_t packet = {           \
        .size=0,                          \
        .code=CODE,                       \
        .data=(uint8_t *)data,            \
    };                                    \
    MultiWii_send(fd, &packet);           \
    packet.size = sizeof(*data);          \
    MultiWii_recv(fd, &packet);           \
    if (packet.size != sizeof(*data))     \
      fprintf(stderr, "Packet size mismatch: %d != %ld\n", packet.size, sizeof(*data)); \
    if (packet.code != CODE)     \
      fprintf(stderr, "Packet code mismatch: %d != %d\n", packet.code, CODE); \
    if (packet.checksum)     \
      fprintf(stderr, "Packet checksum failed: %d\n", packet.checksum); \
}
#endif

namespace mecanum_hardware
{
typedef struct MSP_IDENT_s {
  uint8_t VERSION;
  uint8_t MULTITYPE;
  uint8_t MSP_VERSION;
  uint32_t capability;
} MSP_IDENT_t;

__MSP_RECV__(IDENT, 100);

typedef struct MSP_STATUS_s {
  uint16_t cycleTime;
  uint16_t i2c_errors_count;
  uint16_t sensor;
  uint32_t flag;
  uint8_t global_conf_currentSet;
} MSP_STATUS_t;

__MSP_RECV__(STATUS, 101);

typedef struct MSP_RAW_IMU_s {
  int16_t accX;
  int16_t accY;
  int16_t accZ;
  int16_t gyrX;
  int16_t gyrY;
  int16_t gyrZ;
  int16_t magX;
  int16_t magY;
  int16_t magZ;
} MSP_RAW_IMU_t;

__MSP_RECV__(RAW_IMU, 102);

typedef struct MSP_SERVO_s {
  uint16_t servo[8][2];
} MSP_SERVO_t;

__MSP_RECV__(SERVO, 103);

typedef struct MSP_MOTOR_s {
  uint16_t motor[8];
} MSP_MOTOR_t;

__MSP_RECV__(MOTOR, 104);

typedef struct MSP_RC_s {
  uint16_t ROLL;
  uint16_t PITCH;
  uint16_t YAW;
  uint16_t THROTTLE;
  uint16_t AUX1;
  uint16_t AUX2;
  uint16_t AUX3;
  uint16_t AUX4;
  uint16_t AUX5;
  uint16_t AUX6;
  uint16_t AUX7;
  uint16_t AUX8;
  uint16_t AUX9;
  uint16_t AUX10;
  uint16_t AUX11;
  uint16_t AUX12;
} MSP_RC_t;

__MSP_RECV__(RC, 105);

typedef struct MSP_RAW_GPS_s {
  uint8_t GPS_FIX;
  uint8_t GPS_numSat;
  uint32_t GPS_coord_LAT;
  uint32_t GPS_coord_LON;
  uint16_t GPS_altitude;
  uint16_t GPS_speed;
  uint16_t GPS_ground_course;
} MSP_RAW_GPS_t;

__MSP_RECV__(RAW_GPS, 106);

typedef struct MSP_COMP_GPS_s {
  uint16_t GPS_distanceToHome;
  uint16_t GPS_directionToHome;
  uint8_t GPS_update;
} MSP_COMP_GPS_t;

__MSP_RECV__(COMP_GPS, 107);

typedef struct MSP_ATTITUDE_s {
  int16_t angx;
  int16_t angy;
  int16_t heading;
} MSP_ATTITUDE_t;

__MSP_RECV__(ATTITUDE, 108);

typedef struct MSP_ALTITUDE_s {
  int32_t EstAlt;
  int16_t vario;
} MSP_ALTITUDE_t;

__MSP_RECV__(ALTITUDE, 109);

typedef struct MSP_ANALOG_s {
  uint8_t vbat;
  uint16_t intPowerMeterSum;
  uint16_t rssi;
  uint16_t amperage;
} MSP_ANALOG_t;

__MSP_RECV__(ANALOG, 110);

typedef struct MSP_RC_TUNING_s {
  uint8_t byteRC_RATE;
  uint8_t byteRC_EXPO;
  uint8_t byteRollPitchRate;
  uint8_t byteYawRate;
  uint8_t byteDynThrPID;
  uint8_t byteThrottle_MID;
  uint8_t byteThrottle_EXPO;
} MSP_RC_TUNING_t;

__MSP_RECV__(RC_TUNING, 111);

typedef struct MSP_PID_s {
  uint8_t A_ROLL;
  uint8_t A_PITCH;
  uint8_t A_YAW;
  uint8_t A_ALT;
  uint8_t A_POS;
  uint8_t A_POSR;
  uint8_t A_NAVR;
  uint8_t A_LEVEL;
  uint8_t A_MAG;
  uint8_t A_VEL;
  uint8_t B_ROLL;
  uint8_t B_PITCH;
  uint8_t B_YAW;
  uint8_t B_ALT;
  uint8_t B_POS;
  uint8_t B_POSR;
  uint8_t B_NAVR;
  uint8_t B_LEVEL;
  uint8_t B_MAG;
  uint8_t B_VEL;
  uint8_t C_ROLL;
  uint8_t C_PITCH;
  uint8_t C_YAW;
  uint8_t C_ALT;
  uint8_t C_POS;
  uint8_t C_POSR;
  uint8_t C_NAVR;
  uint8_t C_LEVEL;
  uint8_t C_MAG;
  uint8_t C_VEL;
} MSP_PID_t;

__MSP_RECV__(PID, 112);

typedef struct MSP_BOX_s {
} MSP_BOX_t;

__MSP_RECV__(BOX, 113);

typedef struct MSP_MISC_s {
  uint16_t intPowerTrigger1;
  uint16_t conf_minthrottle;
  uint16_t MAXTHROTTLE;
  uint16_t MINCOMMAND;
  uint16_t conf_failsafe_throttle;
  uint16_t plog_arm;
  uint32_t plog_lifetime;
  uint16_t conf_mag_declination;
  uint8_t conf_vbatscale;
  uint8_t conf_vbatlevel_warn1;
  uint8_t conf_vbatlevel_warn2;
  uint8_t conf_vbatlevel_crit;
} MSP_MISC_t;

__MSP_RECV__(MISC, 114);

typedef struct MSP_MOTOR_PINS_s {
  uint8_t pin[8];
} MSP_MOTOR_PINS_t;

__MSP_RECV__(MOTOR_PINS, 115);

typedef struct MSP_SET_RAW_RC_s {
  uint16_t ROLL;
  uint16_t PITCH;
  uint16_t YAW;
  uint16_t THROTTLE;
  uint16_t AUX1;
  uint16_t AUX2;
  uint16_t AUX3;
  uint16_t AUX4;
  uint16_t AUX5;
  uint16_t AUX6;
  uint16_t AUX7;
  uint16_t AUX8;
  uint16_t AUX9;
  uint16_t AUX10;
  uint16_t AUX11;
  uint16_t AUX12;
} MSP_SET_RAW_RC_t;

__MSP_SEND__(SET_RAW_RC, 200);

typedef struct MSP_SET_RAW_GPS_s {
  uint8_t GPS_FIX;
  uint8_t GPS_numSat;
  uint32_t GPS_coord_LAT;
  uint32_t GPS_coord_LON;
  uint16_t GPS_altitude;
  uint16_t GPS_speed;
} MSP_SET_RAW_GPS_t;

__MSP_SEND__(SET_RAW_GPS, 201);

typedef struct MSP_SET_PID_s {
  uint8_t A_ROLL;
  uint8_t A_PITCH;
  uint8_t A_YAW;
  uint8_t A_ALT;
  uint8_t A_POS;
  uint8_t A_POSR;
  uint8_t A_NAVR;
  uint8_t A_LEVEL;
  uint8_t A_MAG;
  uint8_t A_VEL;
  uint8_t B_ROLL;
  uint8_t B_PITCH;
  uint8_t B_YAW;
  uint8_t B_ALT;
  uint8_t B_POS;
  uint8_t B_POSR;
  uint8_t B_NAVR;
  uint8_t B_LEVEL;
  uint8_t B_MAG;
  uint8_t B_VEL;
  uint8_t C_ROLL;
  uint8_t C_PITCH;
  uint8_t C_YAW;
  uint8_t C_ALT;
  uint8_t C_POS;
  uint8_t C_POSR;
  uint8_t C_NAVR;
  uint8_t C_LEVEL;
  uint8_t C_MAG;
  uint8_t C_VEL;
} MSP_SET_PID_t;

__MSP_SEND__(SET_PID, 202);

typedef struct MSP_SET_BOX_s {
} MSP_SET_BOX_t;

__MSP_SEND__(SET_BOX, 203);

typedef struct MSP_SET_RC_TUNING_s {
  uint8_t byteRC_RATE;
  uint8_t byteRC_EXPO;
  uint8_t byteRollPitchRate;
  uint8_t byteYawRate;
  uint8_t byteDynThrPID;
  uint8_t byteThrottle_MID;
  uint8_t byteThrottle_EXPO;
} MSP_SET_RC_TUNING_t;

__MSP_SEND__(SET_RC_TUNING, 204);

typedef struct MSP_ACC_CALIBRATION_s {
} MSP_ACC_CALIBRATION_t;

__MSP_SEND__(ACC_CALIBRATION, 205);

typedef struct MSP_MAG_CALIBRATION_s {
} MSP_MAG_CALIBRATION_t;

__MSP_SEND__(MAG_CALIBRATION, 206);

typedef struct MSP_SET_MISC_s {
  uint16_t intPowerTrigger1;
  uint16_t conf_minthrottle;
  uint16_t MAXTHROTTLE;
  uint16_t MINCOMMAND;
  uint16_t conf_failsafe_throttle;
  uint16_t plog_arm;
  uint32_t plog_lifetime;
  uint16_t conf_mag_declination;
  uint8_t conf_vbatscale;
  uint8_t conf_vbatlevel_warn1;
  uint8_t conf_vbatlevel_warn2;
  uint8_t conf_vbatlevel_crit;
} MSP_SET_MISC_t;

__MSP_SEND__(SET_MISC, 207);

typedef struct MSP_RESET_CONF_s {
} MSP_RESET_CONF_t;

__MSP_SEND__(RESET_CONF, 208);

typedef struct MSP_SET_HEAD_s {
  int16_t magHold;
} MSP_SET_HEAD_t;

__MSP_SEND__(SET_HEAD, 211);

typedef struct MSP_SET_MOTOR_s {
  uint16_t motor[8];
} MSP_SET_MOTOR_t;

__MSP_SEND__(SET_MOTOR, 214);

typedef struct MSP_BIND_s {
} MSP_BIND_t;

__MSP_SEND__(BIND, 240);

typedef struct MSP_EEPROM_WRITE_s {
} MSP_EEPROM_WRITE_t;

__MSP_SEND__(EEPROM_WRITE, 250);

}