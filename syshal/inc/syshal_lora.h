#ifndef _SYSHAL_LORA_H_
#define _SYSHAL_LORA_H_

#include <stdint.h>
#include <stdbool.h>

// Constants.
#define SYSHAL_LORA_NO_ERROR               ( 0)
#define SYSHAL_LORA_ERROR_FRAME            (-1)
#define SYSHAL_LORA_ERROR_UART             (-2)

#define SYSHAL_LORA_GPIO_POWER_ON  (GPIO_EXT1_GPIO1)

#ifndef UART_LORA_BAUDRATE
#define UART_LORA_BAUDRATE (115200)
#endif

typedef struct
{
    uint32_t iTOW;   // GPS time of week of the navigation epoch
    int32_t  lon;    // Longitude
    int32_t  lat;    // Latitude
    int32_t  hMSL;   // Height above mean sea level
    uint32_t hAcc;   // Horizontal accuracy estimate
    uint32_t vAcc;   // Vertical accuracy estimate
} syshal_lora_position;


int syshal_lora_init(void);
int syshal_lora_send_position(syshal_lora_position *position);

#endif /* _SYSHAL_LORA_H_ */
