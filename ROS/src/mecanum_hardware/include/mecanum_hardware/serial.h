#include <termios.h>
#include <unistd.h>
	
int serial_open(const char *port, int baud);