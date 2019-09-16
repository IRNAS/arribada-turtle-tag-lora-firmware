#include "frame.h"

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>

int main(int argc, char **argv)
{
    if (argc != 2) {
        printf("Usage: %s <serial-device>\n", argv[0]);
        return -1;
    }

    // Open serial device.
    int fd = open(argv[1], O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        printf("Failed to open serial device '%s': %s (%d)\n", argv[1], strerror(errno), errno);
        return -1;
    }

    // Configure serial device.
    struct termios serial_tio;
    if (tcgetattr(fd, &serial_tio) < 0) {
        printf("Failed to get configuration for serial device '%s': %s (%d)\n",
            argv[1], strerror(errno), errno);
        close(fd);
        return -1;
    }

    cfmakeraw(&serial_tio);
    cfsetispeed(&serial_tio, B115200);
    cfsetospeed(&serial_tio, B115200);

    if (tcsetattr(fd, TCSAFLUSH, &serial_tio) < 0) {
        printf("Failed to configure serial device '%s': %s (%d)\n",
            argv[1], strerror(errno), errno);
        close(fd);
        return -1;
    }

    int32_t lon = 42;
    int32_t lat = 16;

    for (;;) {
        message_t msg;
        message_init(&msg);
        tlv_gps_location_t loc = {
            .itow = 123,
            .lon = lon,
            .lat = lat,
            .h_msl = 296,
            .h_acc = 1,
            .v_acc = 1,
        };
        message_tlv_add_gps_location(&msg, &loc);

        // Frame message.
        uint8_t frame[64];
        ssize_t frame_size = frame_message(frame, sizeof(frame), &msg);
        if (frame_size < 0) {
            printf("Failed to frame message!\n");
            return -1;
        }

        printf("Sending framed message:\n");
        for (size_t j = 0; j < frame_size; j++) {
            printf("%02X%s", frame[j], (j < frame_size - 1) ? " " : "");
        }
        printf("\n");

        if (write(fd, frame, (size_t) frame_size) != frame_size) {
            printf("Error writing to serial device: %s (%d)\n", strerror(errno), errno);
            return -1;
        }

        // Increment lat/long.
        lat = (lat + 1) % 90;
        lon = (lon + 1) % 90;

        sleep(10);
    }

    close(fd);
}
