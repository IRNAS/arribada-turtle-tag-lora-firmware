#include "frame.h"

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>

static void incoming_message_handler(const message_t *msg)
{
    message_print(msg);
    printf("\n");
}

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

    // Initialize parser.
    parser_t parser;
    frame_parser_init(&parser);
    parser.handler = incoming_message_handler;

    // Receive messages from the serial device.
    for (;;) {
        char buffer[64];
        ssize_t bytes = read(fd, buffer, sizeof(buffer));
        if (bytes > 0) {
            frame_parser_push_buffer(&parser, buffer, (size_t) bytes);
        }
    }

    close(fd);
}
