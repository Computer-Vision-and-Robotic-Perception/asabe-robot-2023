// MultiWii serial protocol
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include "MultiWii.h"

static const char
    preamble_send[] = "$M<",
    preamble_recv[] = "$M>";

void MultiWii_send(int fd, MultiWiiPacket_t *packet) {
    packet->checksum = packet->size ^ packet->code;
    uint8_t * const end = packet->data + packet->size;
    for (uint8_t *ptr = packet->data; ptr < end; ptr++) {
        packet->checksum ^= *ptr;
    }
    write(fd, preamble_send, sizeof(preamble_send) - 1);
    write(fd, (void *)packet, 2);
    write(fd, packet->data, packet->size);
    write(fd, (void *)&(packet->checksum), 1);
}

/*
 *  Warning:
 *  code and checksum should be checked by caller
 */
void MultiWii_recv(int fd, MultiWiiPacket_t *packet) {
    // Save expected packet size
    const uint8_t expected_size = packet->size;
    // Wait for preamble
    const char *preamble = preamble_recv;
    while (*preamble) {
        char c;
        read(fd, &c, 1);
        if (c == *preamble) {
            preamble++;
        } else {
            preamble = preamble_recv;
        }
    }
    // Preamble has been verified, read packet header
    read(fd, (void *)packet, 2);
    // Read rest of packet data and checksum
    ssize_t bytes_to_read = packet->size + 1;
    uint8_t *data = malloc(bytes_to_read);
    do {
        ssize_t bytes = read(fd, data, bytes_to_read);
        if (bytes < 0) {
            fprintf(stderr, "Error reading from serial: %ld\n", bytes);
            exit(1);
        }
        bytes_to_read -= bytes;
    } while (bytes_to_read > 0);
    // Verify checksum
    packet->checksum = packet->size ^ packet->code;
    for (uint8_t *ptr = data; ptr <= data + packet->size; ptr++) {
        packet->checksum ^= *ptr;
    }
    // Clean existing data in packet data
    memset(packet->data, 0, expected_size);
    // Handle overflow or underflow, if exists
    if (packet->size < expected_size) {
        // underflow
        memcpy(packet->data, data, packet->size);
    } else {
        // normal or overflow
        memcpy(packet->data, data, expected_size);
    }
    // Free allocated memory
    free(data);
}
