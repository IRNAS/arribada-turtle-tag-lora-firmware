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
    SYSHAL_GPS_EVENT_LOCK_MADE,
    SYSHAL_GPS_EVENT_LOCK_LOST,
    SYSHAL_GPS_EVENT_NEW_DATA
} syshal_gps_event_t;

void syshal_gps_init(void);
void syshal_gps_shutdown(void);
void syshal_gps_get_location(uint32_t * iTOW, int32_t * longitude, int32_t * latitude, int32_t * height);
void syshal_gps_tick(void);

void syshal_gps_callback(syshal_gps_event_t event);

#endif /* _SYSHAL_GPS_H_ */