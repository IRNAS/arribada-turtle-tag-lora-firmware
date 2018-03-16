/* config_if.h - Configuration interface abstraction layer. This is used
 * to homogenise the USB and BLE syshals for seamless switching between them
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

#ifndef _CONFIG_IF_H_
#define _CONFIG_IF_H_

#include <stdint.h>
#include <stdbool.h>

// Constants
#define CONFIG_IF_NO_ERROR           	 ( 0)
#define CONFIG_IF_ERROR_BUSY         	 (-1)
#define CONFIG_IF_ERROR_FAIL         	 (-2)
#define CONFIG_IF_ERROR_DISCONNECTED 	 (-3)
#define CONFIG_IF_BUFFER_TOO_SMALL   	 (-4)
#define CONFIG_IF_ERROR_INVALID_SIZE     (-5)
#define CONFIG_IF_ERROR_INVALID_INSTANCE (-6)
#define CONFIG_IF_ALREADY_CONFIGURED     (-7)

typedef enum
{
    CONFIG_IF_EVENT_SEND_COMPLETE, //  BLE or USB send complete
    CONFIG_IF_EVENT_RECEIVE_COMPLETE, // BLE or USB receive complete, the length of data received should be passed back as part of config_if_event_t
    CONFIG_IF_EVENT_CONNECTED, // maps to BLE connection or USB enumeration complete
    CONFIG_IF_EVENT_DISCONNECTED, // maps to BLE disconnection or USB host disconnection
} config_if_event_id_t;

typedef struct
{
    config_if_event_id_t id;
    uint8_t * buffer;
    uint32_t size;
} config_if_event_t;

typedef enum
{
    // GPS
    CONFIG_IF_TAG_GPS_LOG_POSITION_ENABLE = 0x0000, // Enable logging of position information.
    CONFIG_IF_TAG_GPS_LOG_TTFF_ENABLE,              // Enable logging of time to first fix (TTFF)
    CONFIG_IF_TAG_GPS_TRIGGER_MODE,                 // Mode shall allow for continuous operation or external switch triggered operation.
    CONFIG_IF_TAG_GPS_UART_BAUD_RATE,               // The UART baud rate to be used for communications with the M8N GPS device.

    // Real time crystal
    CONFIG_IF_TAG_RTC_SYNC_TO_GPS_ENABLE = 0x0600, // If set, the RTC will sync to GPS time
    CONFIG_IF_TAG_RTC_CURRENT_DATE_AND_TIME,       // Current date and time

    // Logging
    CONFIG_IF_TAG_LOGGING_ENABLE = 0x0100,              // Controls whether the global logging function is enabled or disabled.
    CONFIG_IF_TAG_LOGGING_BYTES_WRITTEN,                // Total number of bytes used in configuration file.
    CONFIG_IF_TAG_LOGGING_FILE_SIZE,                    // Maximum file size allowed for logging. This is set when the logging file is created.
    CONFIG_IF_TAG_LOGGING_FILE_TYPE,                    // Indicates fill or circular mode. This is set when the logging file is created.
    CONFIG_IF_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE, // If set, the logging engine should attempt to group multiple sensor readings together into a single log entry.
    CONFIG_IF_TAG_LOGGING_START_END_SYNC_ENABLE,        // If set, each log entry shall be framed with a start and end sync. This is set when the logging file is created.
    CONFIG_IF_TAG_LOGGING_DATE_TIME_STAMP_ENABLE,       // If set, the RTC date & time shall be logged with each long entry.
    CONFIG_IF_TAG_LOGGING_HIGH_RESOLUTION_TIMER_ENABLE, // If set, the HRT shall be logged with each long entry.

    // Accelerometer
    CONFIG_IF_TAG_AXL_LOG ENABLE = 0x0200,    // The AXL shall be enabled for logging.
    CONFIG_IF_TAG_AXL_CONFIG,                 // G-force setting, sensitivity mode, etc.
    CONFIG_IF_TAG_AXL_G_FORCE_HIGH_THRESHOLD, // The X2+Y2+Z2 vector magnitude threshold.
    CONFIG_IF_TAG_AXL_SAMPLE_RATE,            // Number of times per second to perform a reading of the AXL.
    CONFIG_IF_TAG_AXL_MODE,                   // 0=>PERIODIC, 3=>TRIGGER_ABOVE

    // Pressure sensor
    CONFIG_IF_TAG_PRESSURE_SENSOR_LOG_ENABLE = 0x0300,  // The pressure sensor shall be enabled for logging.
    CONFIG_IF_TAG_PRESSURE_SAMPLE_RATE,                 // Number of times per second to perform a reading of the pressure sensor.
    CONFIG_IF_TAG_PRESSURE_LOW_THRESHOLD,               // Low threshold setting.
    CONFIG_IF_TAG_PRESSURE_HIGH_THRESHOLD,              // High threshold setting.
    CONFIG_IF_TAG_PRESSURE_MODE,                        // 0=>PERIODIC, 1=>TRIGGER_BELOW, 2=>TRIGGER_BETWEEN, 3=>TRIGGER_ABOVE

    // Temperature sensor
    CONFIG_IF_TAG_TEMP_SENSOR_LOG_ENABLE = 0x0700,  // The sensor shall be enabled for logging.
    CONFIG_IF_TAG_TEMP_SENSOR_SAMPLE_RATE,          // Number of times per second to perform a reading of the sensor.
    CONFIG_IF_TAG_TEMP_SENSOR_LOW_THRESHOLD,        // Low threshold setting.
    CONFIG_IF_TAG_TEMP_SENSOR_HIGH_THRESHOLD,       // High threshold setting.
    CONFIG_IF_TAG_TEMP_SENSOR_MODE,                 // 0=>PERIODIC, 1=>TRIGGER_BELOW, 2=>TRIGGER_BETWEEN, 3=>TRIGGER_ABOVE

    // System
    CONFIG_IF_TAG_SYSTEM_DEVICE_IDENTIFIER = 0x0400, // Unique device identifier.

    // Bluetooth
    CONFIG_IF_TAG_BLUETOOTH_UUID = 0x0500,                      // The UUID should uniquely identify the bluetooth function.
    CONFIG_IF_TAG_BLUETOOTH_BEACON_ENABLE,                      // Beacon function is enabled.
    CONFIG_IF_TAG_BLUETOOTH_BEACON_GEO_FENCE_TRIGGER_LOCATION,  // Beacon function is only enabled at the specified geo-fence location.
    CONFIG_IF_TAG_BLUETOOTH_BEACON_ADVERTISING_INTERVAL,        // The beacon advertising interval expressed in milliseconds.
    CONFIG_IF_TAG_BLUETOOTH_BEACON_ADVERTISING_CONFIGURATION,   // TBD - for future expansion of the advertising payload to convey in the beacon.
} config_if_tags_t;

typedef enum
{
    CONFIG_IF_USB,
    CONFIG_IF_BLE
} config_if_backend_t;

int config_if_init(config_if_backend_t backend);
int config_if_term(void);
int config_if_send(uint8_t * data, uint32_t size);
int config_if_receive(uint8_t * data, uint32_t size);
int config_if_event_handler(config_if_event_t * event);

#endif /* _CONFIG_IF_H_ */