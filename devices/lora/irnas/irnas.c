#include <string.h>

#include "syshal_uart.h"
#include "syshal_lora.h"

#include "irnas.h"
#include "message.h"
#include "frame.h"

void syshal_lora_init(void) {
    // Create initialization message.
    message_t msg;
    message_init(&msg);
    message_tlv_add(&msg, TLV_INIT, 0, NULL);

    uint8_t frame[16];
    ssize_t size = frame_message(frame, sizeof(frame), &msg);
    if (size < 0) {
        // TODO: Handle errors.
        return;
    }

    syshal_uart_send(UART_3, frame, size);
}

void syshal_lora_send_position(syshal_lora_position *position) {
    // Create gps position message.
    message_t msg;
    message_init(&msg);

    tlv_gps_location_t loc;
    loc.itow = position->iTOW;
    loc.lon = position->lon;
    loc.lat = position->lat;
    loc.h_msl = position->hMSL;
    loc.h_acc = position->hAcc;
    loc.v_acc = position->vAcc;
    message_tlv_add_gps_location(&msg, &loc);

    uint8_t frame[64];
    ssize_t size = frame_message(frame, sizeof(frame), &msg);
    if (size < 0) {
        // TODO: Handle errors.
        return;
    }

    syshal_uart_send(UART_3, frame, size);
}
