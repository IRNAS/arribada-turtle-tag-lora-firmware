#include <string.h>

#include "irnas.h"
#include "syshal_uart.h"
#include "syshal_lora.h"

void syshal_lora_init(void) {
    // TODO: Define message format.
    syshal_uart_send(UART_3, (uint8_t*) "init\n\r", 6);
}

void syshal_lora_send_position(syshal_lora_position *position) {
    // TODO: Define message format.
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "gps.location %d,%d,%d,%d,%d,%d\n\r",
        (int) position->iTOW,
        (int) position->lon,
        (int) position->lat,
        (int) position->hMSL,
        (int) position->hAcc,
        (int) position->vAcc
    );

    syshal_uart_send(UART_3, (uint8_t*) buffer, strlen(buffer));
}
