/* syshal_gps.h - HAL for gps device
 *
 * Copyright (C) 2018 Arribada
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef _SYSHAL_GPS_H_
#define _SYSHAL_GPS_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    SYSHAL_GPS_EVENT_POSLLH,
    SYSHAL_GPS_EVENT_STATUS
} syshal_gps_event_id_t;

typedef struct
{
    uint32_t iTOW;
    uint8_t  gpsFix;
    uint8_t  flags;
    uint8_t  fixStat;
    uint8_t  flags2;
    uint32_t ttff;
    uint32_t msss;
} syshal_gps_event_status_t;

typedef struct
{
    uint32_t iTOW;   // GPS time of week of the navigation epoch
    int32_t  lon;    // Longitude
    int32_t  lat;    // Latitude
    int32_t  height; // Height above ellipsoid
    int32_t  hMSL;   // Height above mean sea level
    uint32_t hAcc;   // Horizontal accuracy estimate
    uint32_t vAcc;   // Vertical accuracy estimate
} syshal_gps_event_pos_llh_t;

typedef struct
{
    syshal_gps_event_id_t event_id;
    union
    {
        syshal_gps_event_status_t  status;
        syshal_gps_event_pos_llh_t location;
    } event_data;
} syshal_gps_event_t;

void syshal_gps_init(void);
void syshal_gps_shutdown(void);
void syshal_gps_wake_up(void);
void syshal_gps_tick(void);

void syshal_gps_callback(syshal_gps_event_t event);

#endif /* _SYSHAL_GPS_H_ */