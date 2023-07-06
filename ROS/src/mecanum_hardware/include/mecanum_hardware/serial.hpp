#include <termios.h>
#include <unistd.h>

namespace mecanum_hardware
{
int serial_open(const char *port, int baud);
}
