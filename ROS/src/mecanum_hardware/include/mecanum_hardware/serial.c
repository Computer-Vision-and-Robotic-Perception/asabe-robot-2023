#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <stdio.h>
#include "serial.h"

#define ERROR(fmt, ...) fprintf(stderr, fmt, __VA_ARGS__)

int set_interface_attribs(int fd, int speed, int parity) {
    struct termios tty;
    if(tcgetattr(fd, &tty) != 0) {
      ERROR("error %d from tcgetattr", errno);
      return -1;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    // 8-bit chars
    tty.c_cflag =(tty.c_cflag & ~CSIZE) | CS8;
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    // disable break processing
    tty.c_iflag &= ~IGNBRK;
    // no signaling chars, no echo,
    // no canonical processing
    tty.c_lflag = 0;
    // no remapping, no delays
    tty.c_oflag = 0;
    // read doesn't block
    tty.c_cc[VMIN]  = 0;
    // 0.5 seconds read timeout
    tty.c_cc[VTIME] = 5;
    // shut off xon/xoff ctrl
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    // ignore modem controls,
    // enable reading
    tty.c_cflag |= (CLOCAL | CREAD);
    // shut off parity
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif
    // Set the attributes
    if(tcsetattr(fd, TCSANOW, &tty) != 0) {
        ERROR("error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}

void set_blocking(int fd, int should_block) {
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if(tcgetattr(fd, &tty) != 0) {
        ERROR("error %d from tggetattr", errno);
        return;
    }
    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    // 0.5 seconds read timeout
    tty.c_cc[VTIME] = 5;
    if(tcsetattr(fd, TCSANOW, &tty) != 0)
        ERROR("error %d setting term attributes", errno);
}


int serial_open(const char *port, int baud) {
  int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
  if(fd < 0) {
      ERROR("error %d opening %s: %s", errno, port, strerror(errno));
      return -1;
  }
  // set speed to baud, 8n1 (no parity)
  set_interface_attribs(fd, baud, 0);
  // set no blocking
  set_blocking(fd, 0);
  return fd;
}