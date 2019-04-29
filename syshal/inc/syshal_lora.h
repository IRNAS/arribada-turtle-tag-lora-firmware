#ifndef _SYSHAL_LORA_H_
#define _SYSHAL_LORA_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint32_t iTOW;   // GPS time of week of the navigation epoch
    int32_t  lon;    // Longitude
    int32_t  lat;    // Latitude
    int32_t  hMSL;   // Height above mean sea level
    uint32_t hAcc;   // Horizontal accuracy estimate
    uint32_t vAcc;   // Vertical accuracy estimate
} syshal_lora_position;


void syshal_lora_init(void);
void syshal_lora_send_position(syshal_lora_position *position);

#endif /* _SYSHAL_LORA_H_ */