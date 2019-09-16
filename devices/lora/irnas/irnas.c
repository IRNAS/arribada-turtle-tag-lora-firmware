#include <string.h>

#include "syshal_uart.h"
#include "syshal_gpio.h"
#include "syshal_led.h"
#include "syshal_lora.h"

#include "irnas.h"
#include "message.h"
#include "frame.h"

int syshal_lora_init(void)
{
    // Power on device.
    syshal_gpio_init(SYSHAL_LORA_GPIO_POWER_ON);
    syshal_gpio_set_output_high(SYSHAL_LORA_GPIO_POWER_ON);

    if (syshal_uart_change_baud(UART_LORA, UART_LORA_BAUDRATE) != SYSHAL_UART_NO_ERROR) {
        return SYSHAL_LORA_ERROR_UART;
    }

    // Create initialization message.
    message_t msg;
    message_init(&msg);
    message_tlv_add(&msg, TLV_INIT, 0, NULL);

    uint8_t frame[16];
    ssize_t size = frame_message(frame, sizeof(frame), &msg);
    if (size <= 0) {
        return SYSHAL_LORA_ERROR_FRAME;
    }

    int err = syshal_uart_send(UART_LORA, frame, size);
    if (err != SYSHAL_UART_NO_ERROR) {
        return SYSHAL_LORA_ERROR_UART;
    }

    return SYSHAL_LORA_NO_ERROR;
}

int syshal_lora_send_position(syshal_lora_position *position)
{
    // Create gps position message.
    message_t msg;
    message_init(&msg);

    tlv_gps_location_t loc;
    loc.itow = position->itow;
    loc.lon = position->lon;
    loc.lat = position->lat;
    loc.h_msl = position->h_msl;
    loc.h_acc = position->h_acc;
    loc.v_acc = position->v_acc;
    loc.timestamp = position->timestamp;
    message_tlv_add_gps_location(&msg, &loc);

    uint8_t frame[64];
    ssize_t size = frame_message(frame, sizeof(frame), &msg);
    if (size <= 0) {
        return SYSHAL_LORA_ERROR_FRAME;
    }

    if (syshal_uart_send(UART_LORA, frame, size) != SYSHAL_UART_NO_ERROR) {
        return SYSHAL_LORA_ERROR_UART;
    }

    return SYSHAL_LORA_NO_ERROR;
}
