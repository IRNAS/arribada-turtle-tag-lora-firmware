/* sys_config.c - Configuration data and tag accessors/mutators
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

#include "sys_config.h"
#include <string.h>
#include "debug.h"

sys_config_t sys_config; // Configuration data stored in RAM

// Lookup table for checking tag ID is valid and also iterating
static const uint16_t sys_config_lookup_priv[SYS_CONFIG_TAG_TOTAL_NUMBER] =
{
    SYS_CONFIG_TAG_GPS_LOG_POSITION_ENABLE,
    SYS_CONFIG_TAG_GPS_LOG_TTFF_ENABLE,
    SYS_CONFIG_TAG_GPS_TRIGGER_MODE,
    SYS_CONFIG_TAG_GPS_UART_BAUD_RATE,
    SYS_CONFIG_TAG_RTC_SYNC_TO_GPS_ENABLE,
    SYS_CONFIG_TAG_RTC_CURRENT_DATE_AND_TIME,
    SYS_CONFIG_TAG_LOGGING_ENABLE,
    SYS_CONFIG_TAG_LOGGING_BYTES_WRITTEN,
    SYS_CONFIG_TAG_LOGGING_FILE_SIZE,
    SYS_CONFIG_TAG_LOGGING_FILE_TYPE,
    SYS_CONFIG_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE,
    SYS_CONFIG_TAG_LOGGING_START_END_SYNC_ENABLE,
    SYS_CONFIG_TAG_LOGGING_DATE_TIME_STAMP_ENABLE,
    SYS_CONFIG_TAG_LOGGING_HIGH_RESOLUTION_TIMER_ENABLE,
    SYS_CONFIG_TAG_AXL_LOG_ENABLE,
    SYS_CONFIG_TAG_AXL_CONFIG,
    SYS_CONFIG_TAG_AXL_G_FORCE_HIGH_THRESHOLD,
    SYS_CONFIG_TAG_AXL_SAMPLE_RATE,
    SYS_CONFIG_TAG_AXL_MODE,
    SYS_CONFIG_TAG_PRESSURE_SENSOR_LOG_ENABLE,
    SYS_CONFIG_TAG_PRESSURE_SAMPLE_RATE,
    SYS_CONFIG_TAG_PRESSURE_LOW_THRESHOLD,
    SYS_CONFIG_TAG_PRESSURE_HIGH_THRESHOLD,
    SYS_CONFIG_TAG_PRESSURE_MODE,
    SYS_CONFIG_TAG_TEMP_SENSOR_LOG_ENABLE,
    SYS_CONFIG_TAG_TEMP_SENSOR_SAMPLE_RATE,
    SYS_CONFIG_TAG_TEMP_SENSOR_LOW_THRESHOLD,
    SYS_CONFIG_TAG_TEMP_SENSOR_HIGH_THRESHOLD,
    SYS_CONFIG_TAG_TEMP_SENSOR_MODE,
    SYS_CONFIG_TAG_SYSTEM_DEVICE_IDENTIFIER,
    SYS_CONFIG_TAG_BLUETOOTH_UUID,
    SYS_CONFIG_TAG_BLUETOOTH_BEACON_ENABLE,
    SYS_CONFIG_TAG_BLUETOOTH_BEACON_GEO_FENCE_TRIGGER_LOCATION,
    SYS_CONFIG_TAG_BLUETOOTH_BEACON_ADVERTISING_INTERVAL,
    SYS_CONFIG_TAG_BLUETOOTH_BEACON_ADVERTISING_CONFIGURATION,
};

/**
 * @brief      Gets the pointer to the data of the given tag
 *
 * @param[in]  tag   The configuration tag
 * @param[out] data  The pointer to the tags data
 *
 * @return     The length of the given configuration tag
 * @return     SYS_CONFIG_ERROR_INVALID_TAG if the given tag is invalid
 */
int sys_config_get_data_ptr_priv(uint16_t tag, void ** data)
{
    uint32_t len = 0;

    switch (tag)
    {
        case SYS_CONFIG_TAG_GPS_LOG_POSITION_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_gps_log_position_enable_t);
            *data = &sys_config.sys_config_gps_log_position_enable.contents;
            break;

        case SYS_CONFIG_TAG_GPS_LOG_TTFF_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_gps_log_ttff_enable_t);
            *data = &sys_config.sys_config_gps_log_ttff_enable.contents;
            break;

        case SYS_CONFIG_TAG_GPS_TRIGGER_MODE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_gps_trigger_mode_t);
            *data = &sys_config.sys_config_gps_trigger_mode.contents;
            break;

        case SYS_CONFIG_TAG_GPS_UART_BAUD_RATE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_gps_uart_baud_rate_t);
            *data = &sys_config.sys_config_gps_uart_baud_rate.contents;
            break;

        case SYS_CONFIG_TAG_RTC_SYNC_TO_GPS_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_rtc_sync_to_gps_enable_t);
            *data = &sys_config.sys_config_rtc_sync_to_gps_enable.contents;
            break;

        case SYS_CONFIG_TAG_RTC_CURRENT_DATE_AND_TIME:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_rtc_current_date_and_time_t);
            *data = &sys_config.sys_config_rtc_current_date_and_time.contents;
            break;

        case SYS_CONFIG_TAG_LOGGING_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_enable_t);
            *data = &sys_config.sys_config_logging_enable.contents;
            break;

        case SYS_CONFIG_TAG_LOGGING_BYTES_WRITTEN:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_bytes_written_t);
            *data = &sys_config.sys_config_logging_bytes_written.contents;
            break;

        case SYS_CONFIG_TAG_LOGGING_FILE_SIZE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_file_size_t);
            *data = &sys_config.sys_config_logging_file_size.contents;
            break;

        case SYS_CONFIG_TAG_LOGGING_FILE_TYPE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_file_type_t);
            *data = &sys_config.sys_config_logging_file_type.contents;
            break;

        case SYS_CONFIG_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_group_sensor_readings_enable_t);
            *data = &sys_config.sys_config_logging_group_sensor_readings_enable.contents;
            break;

        case SYS_CONFIG_TAG_LOGGING_START_END_SYNC_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_start_end_sync_enable_t);
            *data = &sys_config.sys_config_logging_start_end_sync_enable.contents;
            break;

        case SYS_CONFIG_TAG_LOGGING_DATE_TIME_STAMP_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_date_time_stamp_enable_t);
            *data = &sys_config.sys_config_logging_date_time_stamp_enable.contents;
            break;

        case SYS_CONFIG_TAG_LOGGING_HIGH_RESOLUTION_TIMER_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_high_resolution_timer_enable_t);
            *data = &sys_config.sys_config_logging_high_resolution_timer_enable.contents;
            break;

        case SYS_CONFIG_TAG_AXL_LOG_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_axl_log_enable_t);
            *data = &sys_config.sys_config_axl_log_enable.contents;
            break;

        case SYS_CONFIG_TAG_AXL_CONFIG:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_axl_config_t);
            *data = &sys_config.sys_config_axl_config.contents;
            break;

        case SYS_CONFIG_TAG_AXL_G_FORCE_HIGH_THRESHOLD:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_axl_g_force_high_threshold_t);
            *data = &sys_config.sys_config_axl_g_force_high_threshold.contents;
            break;

        case SYS_CONFIG_TAG_AXL_SAMPLE_RATE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_axl_sample_rate_t);
            *data = &sys_config.sys_config_axl_sample_rate.contents;
            break;

        case SYS_CONFIG_TAG_AXL_MODE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_axl_mode_t);
            *data = &sys_config.sys_config_axl_mode.contents;
            break;

        case SYS_CONFIG_TAG_PRESSURE_SENSOR_LOG_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_pressure_sensor_log_enable_t);
            *data = &sys_config.sys_config_pressure_sensor_log_enable.contents;
            break;

        case SYS_CONFIG_TAG_PRESSURE_SAMPLE_RATE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_pressure_sample_rate_t);
            *data = &sys_config.sys_config_pressure_sample_rate.contents;
            break;

        case SYS_CONFIG_TAG_PRESSURE_LOW_THRESHOLD:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_pressure_low_threshold_t);
            *data = &sys_config.sys_config_pressure_low_threshold.contents;
            break;

        case SYS_CONFIG_TAG_PRESSURE_HIGH_THRESHOLD:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_pressure_high_threshold_t);
            *data = &sys_config.sys_config_pressure_high_threshold.contents;
            break;

        case SYS_CONFIG_TAG_PRESSURE_MODE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_pressure_mode_t);
            *data = &sys_config.sys_config_pressure_mode.contents;
            break;

        case SYS_CONFIG_TAG_TEMP_SENSOR_LOG_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_temp_sensor_log_enable_t);
            *data = &sys_config.sys_config_temp_sensor_log_enable.contents;
            break;

        case SYS_CONFIG_TAG_TEMP_SENSOR_SAMPLE_RATE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_temp_sensor_sample_rate_t);
            *data = &sys_config.sys_config_temp_sensor_sample_rate.contents;
            break;

        case SYS_CONFIG_TAG_TEMP_SENSOR_LOW_THRESHOLD:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_temp_sensor_low_threshold_t);
            *data = &sys_config.sys_config_temp_sensor_low_threshold.contents;
            break;

        case SYS_CONFIG_TAG_TEMP_SENSOR_HIGH_THRESHOLD:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_temp_sensor_high_threshold_t);
            *data = &sys_config.sys_config_temp_sensor_high_threshold.contents;
            break;

        case SYS_CONFIG_TAG_TEMP_SENSOR_MODE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_temp_sensor_mode_t);
            *data = &sys_config.sys_config_temp_sensor_mode.contents;
            break;

        case SYS_CONFIG_TAG_SYSTEM_DEVICE_IDENTIFIER:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_system_device_identifier_t);
            *data = &sys_config.sys_config_system_device_identifier.contents;
            break;

        case SYS_CONFIG_TAG_BLUETOOTH_UUID:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_bluetooth_uuid_t);
            *data = &sys_config.sys_config_bluetooth_uuid.contents;
            break;

        case SYS_CONFIG_TAG_BLUETOOTH_BEACON_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_bluetooth_beacon_enable_t);
            *data = &sys_config.sys_config_bluetooth_beacon_enable.contents;
            break;

        case SYS_CONFIG_TAG_BLUETOOTH_BEACON_GEO_FENCE_TRIGGER_LOCATION:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_bluetooth_beacon_geo_fence_trigger_location_t);
            *data = &sys_config.sys_config_bluetooth_beacon_geo_fence_trigger_location.contents;
            break;

        case SYS_CONFIG_TAG_BLUETOOTH_BEACON_ADVERTISING_INTERVAL:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_bluetooth_beacon_advertising_interval_t);
            *data = &sys_config.sys_config_bluetooth_beacon_advertising_interval.contents;
            break;

        case SYS_CONFIG_TAG_BLUETOOTH_BEACON_ADVERTISING_CONFIGURATION:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_bluetooth_beacon_advertising_configuration_t);
            *data = &sys_config.sys_config_bluetooth_beacon_advertising_configuration.contents;
            break;

        default:
            return SYS_CONFIG_ERROR_INVALID_TAG;
    }

    return len;
}

/**
 * @brief      Set the configuration tag to the given value
 *
 * @param[in]  tag     The configuration tag
 * @param[in]  value   The value to set the tag to
 * @param[in]  length  The length of the data to be written
 *
 * @return     SYS_CONFIG_NO_ERROR on success
 * @return     SYS_CONFIG_ERROR_INVALID_TAG if the given tag is invalid
 * @return     SYS_CONFIG_ERROR_WRONG_SIZE if the given length does not meet the
 *             configuration tag length
 */
int sys_config_set(uint16_t tag, void * value, uint32_t length)
{
    // Get the buffer of the configuration tag in RAM
    void * data = NULL;
    int return_code = sys_config_get_data_ptr_priv(tag, &data);

    if (return_code < 0)
        return return_code;

    if (return_code != length)
        return SYS_CONFIG_ERROR_WRONG_SIZE;

    // Copy data into the configuration tag
#pragma GCC diagnostic push // Suppress data = NULL warning as this cannot happen as the function returns if that is the case
#pragma GCC diagnostic ignored "-Wnonnull"
    memcpy(data, value, length);
#pragma GCC diagnostic pop

    // Set the configuration set tag
    // This is done so we can always tell if a configuration tag has been setup or has been erased or never initialised
    ((sys_config_hdr_t *)data)->set = true;

    return SYS_CONFIG_NO_ERROR;
}

/**
 * @brief      Clear the given configuration tag
 *
 * @param[in]  tag   The configuration tag
 *
 * @return     SYS_CONFIG_NO_ERROR on success
 * @return     SYS_CONFIG_ERROR_INVALID_TAG if the given tag is invalid
 */
int sys_config_unset(uint16_t tag)
{
    // Get the address of the configuration data in RAM
    void * data = NULL;
    int return_code = sys_config_get_data_ptr_priv(tag, &data);

    if (return_code < 0)
        return return_code;

    ((sys_config_hdr_t *)data)->set = false; // Unclear the configuration set tag

    return SYS_CONFIG_NO_ERROR;
}

/**
 * @brief      Gets the value in the given configuration tag
 *
 * @param[in]  tag     The configuration tag
 * @param[out] value   The buffer to retrieve the value into
 *
 * @return     The length of the data read on success
 * @return     SYS_CONFIG_ERROR_INVALID_TAG if the given tag is invalid
 * @return     SYS_CONFIG_ERROR_TAG_NOT_SET if the tag hasn't been set
 */
int sys_config_get(uint16_t tag, void * value)
{
    // Get the address of the configuration data in RAM
    void * data = NULL;
    int return_code = sys_config_get_data_ptr_priv(tag, &data);

    if (return_code < 0)
        return return_code;

    // If this configuration tag hasn't been previously set then return an error
    if (false == ((sys_config_hdr_t *)data)->set)
        return SYS_CONFIG_ERROR_TAG_NOT_SET;

    // Copy the configuration data back
#pragma GCC diagnostic push // Suppress data = NULL warning as this cannot happen as the function returns if that is the case
#pragma GCC diagnostic ignored "-Wnonnull"
    if (NULL != value)
        memcpy(value, data, return_code);
#pragma GCC diagnostic pop

    return return_code;
}

/**
 * @brief      Checks to see if the given tag is valid
 *
 * @param[in]  tag   The configuration tag
 *
 * @return     true if valid
 */
bool sys_config_is_valid(uint16_t tag)
{
    for (uint32_t i = 0; i < SYS_CONFIG_TAG_TOTAL_NUMBER; ++i)
    {
        if (tag == sys_config_lookup_priv[i])
            return true;
    }

    return false;
}

/**
 * @brief      Returns the next tag after the given tag. This is used for
 *             iterating through all the tags
 *
 * @warning    This function assumes the configuration tag given is valid and
 *             will produce spurious output if it is not
 *
 * @param[in]  tag         The configuration tag
 * @param      last_index  Private variable for maintaining state. For the first
 *                         call pass a pointer to a uint32_t set to 0
 *
 * @return     The next tag on success
 * @return     SYS_CONFIG_ERROR_NO_MORE_TAGS if there are no more tags after this
 *             one
 */
int sys_config_iterate(uint16_t tag, uint16_t * last_index)
{
    // Find tag in lookup table
    for (; (*last_index) < SYS_CONFIG_TAG_TOTAL_NUMBER; ++(*last_index))
    {
        if (tag == sys_config_lookup_priv[(*last_index)])
            break;
    }

    (*last_index)++; // Increment to the next tag

    // If this tag is the last one then there are no more tags after it
    if (SYS_CONFIG_TAG_TOTAL_NUMBER == (*last_index))
        return SYS_CONFIG_ERROR_NO_MORE_TAGS;

    // Return the next tag in the list
    return sys_config_lookup_priv[(*last_index)];
}
