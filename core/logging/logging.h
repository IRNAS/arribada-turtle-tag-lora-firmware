/* logging.h - Logging defines
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

#ifndef _LOGGING_H_
#define _LOGGING_H_

#include <stdint.h>

#define LOGGING_SET_HDR(p, i)  \
    (p)->h.id   = i;

#define LOGGING_LOG_START     (0x7E) // Used to synchronize to the start of a log entry
#define LOGGING_LOG_END       (0x7F) // Used to synchronize to the end of a log entry, including an 8-bit parity checksum
#define LOGGING_GPS_POSITION  (0x00) // GPS location
#define LOGGING_GPS_TTFF      (0x01) // GPS time to first fix
#define LOGGING_PRESSURE      (0x02) // Pressure sensor reading
#define LOGGING_AXL_XYZ       (0x03) // Accelerometer X/Y/Z reading
#define LOGGING_DATE_TIME     (0x04) // The date and time retrieved from the RTC
#define LOGGING_HRT           (0x05) // High resolution timer
#define LOGGING_TEMPERATURE   (0x06)
#define LOGGING_SURFACED      (0x07) // Saltwater switch opened event
#define LOGGING_SUBMERGED     (0x08) // Saltwater switch closed event
#define LOGGING_BATTERY       (0x09) // Battery charge state

typedef struct __attribute__((__packed__))
{
    uint8_t id; // Tag ID
} logging_hdr_t;

typedef struct __attribute__((__packed__))
{
    logging_hdr_t h;
} logging_log_start_t;

typedef struct __attribute__((__packed__))
{
    logging_hdr_t h;
} logging_log_end_t;

typedef struct __attribute__((__packed__))
{
    logging_hdr_t h;
    uint32_t iTOW;   // Time since navigation epoch in ms
    int32_t lon;     // Longitude (10^-7)
    int32_t lat;     // Latitude (10^-7)
    int32_t height;  // Height in mm
} logging_gps_position_t;

typedef struct __attribute__((__packed__))
{
    logging_hdr_t h;
    uint32_t ttff; // GPS time to first fix
} logging_gps_ttff_t;

typedef struct __attribute__((__packed__))
{
    logging_hdr_t h;
    int32_t pressure; // Pressure sensor reading
} logging_pressure_t;

typedef struct __attribute__((__packed__))
{
    logging_hdr_t h;
    int16_t x; // X axis
    int16_t y; // Y axis
    int16_t z; // Z axis
} logging_axl_xyz_t;

typedef struct __attribute__((__packed__))
{
    logging_hdr_t h;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} logging_date_time_t;

typedef struct __attribute__((__packed__))
{
    logging_hdr_t h;
    uint64_t us; // High resolution timer in microseconds
} logging_hrt_t;

typedef struct __attribute__((__packed__))
{
    logging_hdr_t h;
} logging_surfaced_t;

typedef struct __attribute__((__packed__))
{
    logging_hdr_t h;
} logging_submerged_t;

typedef struct __attribute__((__packed__))
{
    logging_hdr_t h;
    uint8_t charge;
} logging_battery_t;

#endif /* _LOGGING_H_ */