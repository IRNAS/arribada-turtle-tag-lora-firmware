/* sys_config.h - Configuration data and tag definitions
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

#include <stdint.h>
#include <stdbool.h>

// Constants
#define SYS_CONFIG_NO_ERROR               ( 0)
#define SYS_CONFIG_ERROR_INVALID_TAG      (-1)
#define SYS_CONFIG_ERROR_WRONG_SIZE       (-2)
#define SYS_CONFIG_ERROR_NO_MORE_TAGS     (-3)
#define SYS_CONFIG_ERROR_TAG_NOT_SET      (-4)

#define SYS_CONFIG_MAX_DATA_SIZE (60) // Max size the configuration tag's data can be in bytes

#define SYS_CONFIG_TAG_ID_SIZE (sizeof(uint16_t))
#define SYS_CONFIG_TAG_DATA_SIZE(tag_type) (sizeof(((tag_type *)0)->contents)) // Size of data in tag. We exclude the set member

#define SYS_CONFIG_TAG_TOTAL_NUMBER (35) // Number of configuration tags - WARN: This has to be manually updated

enum
{
    // GPS
    SYS_CONFIG_TAG_GPS_LOG_POSITION_ENABLE = 0x0000, // Enable logging of position information.
    SYS_CONFIG_TAG_GPS_LOG_TTFF_ENABLE,              // Enable logging of time to first fix (TTFF)
    SYS_CONFIG_TAG_GPS_TRIGGER_MODE,                 // Mode shall allow for continuous operation or external switch triggered operation.
    SYS_CONFIG_TAG_GPS_UART_BAUD_RATE,               // The UART baud rate to be used for communications with the M8N GPS device.

    // Real time crystal
    SYS_CONFIG_TAG_RTC_SYNC_TO_GPS_ENABLE = 0x0600, // If set, the RTC will sync to GPS time
    SYS_CONFIG_TAG_RTC_CURRENT_DATE_AND_TIME,       // Current date and time

    // Logging
    SYS_CONFIG_TAG_LOGGING_ENABLE = 0x0100,              // Controls whether the global logging function is enabled or disabled.
    SYS_CONFIG_TAG_LOGGING_BYTES_WRITTEN,                // Total number of bytes used in configuration file.
    SYS_CONFIG_TAG_LOGGING_FILE_SIZE,                    // Maximum file size allowed for logging. This is set when the logging file is created.
    SYS_CONFIG_TAG_LOGGING_FILE_TYPE,                    // Indicates fill or circular mode. This is set when the logging file is created.
    SYS_CONFIG_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE, // If set, the logging engine should attempt to group multiple sensor readings together into a single log entry.
    SYS_CONFIG_TAG_LOGGING_START_END_SYNC_ENABLE,        // If set, each log entry shall be framed with a start and end sync. This is set when the logging file is created.
    SYS_CONFIG_TAG_LOGGING_DATE_TIME_STAMP_ENABLE,       // If set, the RTC date & time shall be logged with each long entry.
    SYS_CONFIG_TAG_LOGGING_HIGH_RESOLUTION_TIMER_ENABLE, // If set, the HRT shall be logged with each long entry.

    // Accelerometer
    SYS_CONFIG_TAG_AXL_LOG_ENABLE = 0x0200,    // The AXL shall be enabled for logging.
    SYS_CONFIG_TAG_AXL_CONFIG,                 // G-force setting, sensitivity mode, etc.
    SYS_CONFIG_TAG_AXL_G_FORCE_HIGH_THRESHOLD, // The X2+Y2+Z2 vector magnitude threshold.
    SYS_CONFIG_TAG_AXL_SAMPLE_RATE,            // Number of times per second to perform a reading of the AXL.
    SYS_CONFIG_TAG_AXL_MODE,                   // 0=>PERIODIC, 3=>TRIGGER_ABOVE

    // Pressure sensor
    SYS_CONFIG_TAG_PRESSURE_SENSOR_LOG_ENABLE = 0x0300,  // The pressure sensor shall be enabled for logging.
    SYS_CONFIG_TAG_PRESSURE_SAMPLE_RATE,                 // Number of times per second to perform a reading of the pressure sensor.
    SYS_CONFIG_TAG_PRESSURE_LOW_THRESHOLD,               // Low threshold setting.
    SYS_CONFIG_TAG_PRESSURE_HIGH_THRESHOLD,              // High threshold setting.
    SYS_CONFIG_TAG_PRESSURE_MODE,                        // 0=>PERIODIC, 1=>TRIGGER_BELOW, 2=>TRIGGER_BETWEEN, 3=>TRIGGER_ABOVE

    // Temperature sensor
    SYS_CONFIG_TAG_TEMP_SENSOR_LOG_ENABLE = 0x0700,  // The sensor shall be enabled for logging.
    SYS_CONFIG_TAG_TEMP_SENSOR_SAMPLE_RATE,          // Number of times per second to perform a reading of the sensor.
    SYS_CONFIG_TAG_TEMP_SENSOR_LOW_THRESHOLD,        // Low threshold setting.
    SYS_CONFIG_TAG_TEMP_SENSOR_HIGH_THRESHOLD,       // High threshold setting.
    SYS_CONFIG_TAG_TEMP_SENSOR_MODE,                 // 0=>PERIODIC, 1=>TRIGGER_BELOW, 2=>TRIGGER_BETWEEN, 3=>TRIGGER_ABOVE

    // System
    SYS_CONFIG_TAG_SYSTEM_DEVICE_IDENTIFIER = 0x0400, // Unique device identifier.

    // Bluetooth
    SYS_CONFIG_TAG_BLUETOOTH_UUID = 0x0500,                      // The UUID should uniquely identify the bluetooth function.
    SYS_CONFIG_TAG_BLUETOOTH_BEACON_ENABLE,                      // Beacon function is enabled.
    SYS_CONFIG_TAG_BLUETOOTH_BEACON_GEO_FENCE_TRIGGER_LOCATION,  // Beacon function is only enabled at the specified geo-fence location.
    SYS_CONFIG_TAG_BLUETOOTH_BEACON_ADVERTISING_INTERVAL,        // The beacon advertising interval expressed in milliseconds.
    SYS_CONFIG_TAG_BLUETOOTH_BEACON_ADVERTISING_CONFIGURATION,   // TBD - for future expansion of the advertising payload to convey in the beacon.
};

typedef struct __attribute__((__packed__))
{
    bool set; // Whether or not this command tag has been set
} sys_config_hdr_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint8_t enable;
    } contents;
} sys_config_gps_log_position_enable_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint8_t enable;
    } contents;
} sys_config_gps_log_ttff_enable_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint8_t mode;
    } contents;
} sys_config_gps_trigger_mode_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint32_t baudrate;
    } contents;
} sys_config_gps_uart_baud_rate_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint8_t enable;
    } contents;
} sys_config_rtc_sync_to_gps_enable_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint8_t day;
        uint8_t month;
        uint16_t year;
        uint8_t hours;
        uint8_t minutes;
        uint8_t seconds;
    } contents;
} sys_config_rtc_current_date_and_time_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint8_t enable;
    } contents;
} sys_config_logging_enable_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint32_t bytes_written;
    } contents;
} sys_config_logging_bytes_written_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint32_t file_size;
    } contents;
} sys_config_logging_file_size_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint8_t file_type;
    } contents;
} sys_config_logging_file_type_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint8_t enable;
    } contents;
} sys_config_logging_group_sensor_readings_enable_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint8_t enable;
    } contents;
} sys_config_logging_start_end_sync_enable_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint8_t enable;
    } contents;
} sys_config_logging_date_time_stamp_enable_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint8_t enable;
    } contents;
} sys_config_logging_high_resolution_timer_enable_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint8_t enable;
    } contents;
} sys_config_axl_log_enable_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint8_t config;
    } contents;
} sys_config_axl_config_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint16_t threshold;
    } contents;
} sys_config_axl_g_force_high_threshold_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint16_t sample_rate;
    } contents;
} sys_config_axl_sample_rate_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint8_t mode;
    } contents;
} sys_config_axl_mode_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint8_t enable;
    } contents;
} sys_config_pressure_sensor_log_enable_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint16_t sample_rate;
    } contents;
} sys_config_pressure_sample_rate_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint16_t threshold;
    } contents;
} sys_config_pressure_low_threshold_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint16_t threshold;
    } contents;
} sys_config_pressure_high_threshold_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint8_t mode;
    } contents;
} sys_config_pressure_mode_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint8_t enable;
    } contents;
} sys_config_temp_sensor_log_enable_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint16_t sample_rate;
    } contents;
} sys_config_temp_sensor_sample_rate_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint16_t threshold;
    } contents;
} sys_config_temp_sensor_low_threshold_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint16_t threshold;
    } contents;
} sys_config_temp_sensor_high_threshold_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint16_t mode;
    } contents;
} sys_config_temp_sensor_mode_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint8_t id[8];
    } contents;
} sys_config_system_device_identifier_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint8_t uuid[16];
    } contents;
} sys_config_bluetooth_uuid_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint8_t enable;
    } contents;
} sys_config_bluetooth_beacon_enable_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint8_t location[12];
    } contents;
} sys_config_bluetooth_beacon_geo_fence_trigger_location_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint16_t interval;
    } contents;
} sys_config_bluetooth_beacon_advertising_interval_t;

typedef struct __attribute__((__packed__))
{
    sys_config_hdr_t hdr;
    struct __attribute__((__packed__))
    {
        uint8_t configuration;
    } contents;
} sys_config_bluetooth_beacon_advertising_configuration_t;

typedef struct __attribute__((__packed__))
{
    sys_config_gps_log_position_enable_t                        sys_config_gps_log_position_enable;
    sys_config_gps_log_ttff_enable_t                            sys_config_gps_log_ttff_enable;
    sys_config_gps_trigger_mode_t                               sys_config_gps_trigger_mode;
    sys_config_gps_uart_baud_rate_t                             sys_config_gps_uart_baud_rate;
    sys_config_rtc_sync_to_gps_enable_t                         sys_config_rtc_sync_to_gps_enable;
    sys_config_rtc_current_date_and_time_t                      sys_config_rtc_current_date_and_time;
    sys_config_logging_enable_t                                 sys_config_logging_enable;
    sys_config_logging_bytes_written_t                          sys_config_logging_bytes_written;
    sys_config_logging_file_size_t                              sys_config_logging_file_size;
    sys_config_logging_file_type_t                              sys_config_logging_file_type;
    sys_config_logging_group_sensor_readings_enable_t           sys_config_logging_group_sensor_readings_enable;
    sys_config_logging_start_end_sync_enable_t                  sys_config_logging_start_end_sync_enable;
    sys_config_logging_date_time_stamp_enable_t                 sys_config_logging_date_time_stamp_enable;
    sys_config_logging_high_resolution_timer_enable_t           sys_config_logging_high_resolution_timer_enable;
    sys_config_axl_log_enable_t                                 sys_config_axl_log_enable;
    sys_config_axl_config_t                                     sys_config_axl_config;
    sys_config_axl_g_force_high_threshold_t                     sys_config_axl_g_force_high_threshold;
    sys_config_axl_sample_rate_t                                sys_config_axl_sample_rate;
    sys_config_axl_mode_t                                       sys_config_axl_mode;
    sys_config_pressure_sensor_log_enable_t                     sys_config_pressure_sensor_log_enable;
    sys_config_pressure_sample_rate_t                           sys_config_pressure_sample_rate;
    sys_config_pressure_low_threshold_t                         sys_config_pressure_low_threshold;
    sys_config_pressure_high_threshold_t                        sys_config_pressure_high_threshold;
    sys_config_pressure_mode_t                                  sys_config_pressure_mode;
    sys_config_temp_sensor_log_enable_t                         sys_config_temp_sensor_log_enable;
    sys_config_temp_sensor_sample_rate_t                        sys_config_temp_sensor_sample_rate;
    sys_config_temp_sensor_low_threshold_t                      sys_config_temp_sensor_low_threshold;
    sys_config_temp_sensor_high_threshold_t                     sys_config_temp_sensor_high_threshold;
    sys_config_temp_sensor_mode_t                               sys_config_temp_sensor_mode;
    sys_config_system_device_identifier_t                       sys_config_system_device_identifier;
    sys_config_bluetooth_uuid_t                                 sys_config_bluetooth_uuid;
    sys_config_bluetooth_beacon_enable_t                        sys_config_bluetooth_beacon_enable;
    sys_config_bluetooth_beacon_geo_fence_trigger_location_t    sys_config_bluetooth_beacon_geo_fence_trigger_location;
    sys_config_bluetooth_beacon_advertising_interval_t          sys_config_bluetooth_beacon_advertising_interval;
    sys_config_bluetooth_beacon_advertising_configuration_t     sys_config_bluetooth_beacon_advertising_configuration;
} sys_config_t;

// Functions
int sys_config_set(uint16_t tag, void * value, uint32_t length);
int sys_config_unset(uint16_t tag);
int sys_config_get(uint16_t tag, void * value);
int sys_config_size(uint16_t tag);
bool sys_config_is_valid(uint16_t tag);
int sys_config_iterate(uint16_t tag, uint16_t * last_index);
