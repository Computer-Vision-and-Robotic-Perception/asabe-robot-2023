// MultiWii serial protocol
#include <stdint.h>
#include <stdlib.h>

typedef struct MultiWiiPacket_s {
  uint8_t size;
  uint8_t code;
  uint8_t *data;
  // Checksum should be 0 for received packet
  uint8_t checksum;
} MultiWiiPacket_t;

void MultiWii_send(int, MultiWiiPacket_t *);
void MultiWii_recv(int, MultiWiiPacket_t *);
