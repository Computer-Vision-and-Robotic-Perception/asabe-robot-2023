#include <stdio.h>
#include "MSP.h"
#include "serial.h"

int main(int argc, char* argv[]) {
    if (argc < 2) {
        printf("Usage: %s [serial port]\n", argv[0]);
        return 1;
    }
    printf("Opening serial port %s\n", argv[1]);
    int fd = serial_open(argv[1], B115200);
    if (fd < 0) {
        printf("Error opening serial port\n");
        return 1;
    }
    printf("Setting motor speeds\n");
    MSP_SET_MOTOR_t motors = {
        .motor={1500, 1500, 1500, 1500}
    };
    MSP_SET_MOTOR(fd, &motors);
    return 0;
}
