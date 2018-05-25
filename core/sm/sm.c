/* sm.c - Main state machine
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

#include <string.h>
#include "bsp.h"
#include "buffer.h"
#include "cexception.h"
#include "cmd.h"
#include "config_if.h"
#include "crc32.h"
#include "debug.h"
#include "exceptions.h"
#include "fs.h"
#include "sm.h"
#include "sys_config.h"
#include "logging.h"
#include "syshal_axl.h"
#include "syshal_batt.h"
#include "syshal_gpio.h"
#include "syshal_gps.h"
#include "syshal_flash.h"
#include "syshal_i2c.h"
#include "syshal_pmu.h"
#include "syshal_rtc.h"
#include "syshal_spi.h"
#include "syshal_switch.h"
#include "syshal_uart.h"
#include "syshal_usb.h"
#include "version.h"

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// MAIN STATES ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static sm_state_t state = SM_STATE_BOOT; // Default starting state

#ifndef DEBUG_DISABLED
static const char * sm_state_str[] =
{
    [SM_STATE_BOOT]                         = "SM_STATE_BOOT",
    [SM_STATE_STANDBY_BATTERY_CHARGING]     = "SM_STATE_STANDBY_BATTERY_CHARGING",
    [SM_STATE_STANDBY_BATTERY_LEVEL_LOW]    = "SM_STATE_STANDBY_BATTERY_LEVEL_LOW",
    [SM_STATE_STANDBY_LOG_FILE_FULL]        = "SM_STATE_STANDBY_LOG_FILE_FULL",
    [SM_STATE_STANDBY_PROVISIONING_NEEDED]  = "SM_STATE_STANDBY_PROVISIONING_NEEDED",
    [SM_STATE_STANDBY_TRIGGER_PENDING]      = "SM_STATE_STANDBY_TRIGGER_PENDING",
    [SM_STATE_PROVISIONING]                 = "SM_STATE_PROVISIONING",
    [SM_STATE_OPERATIONAL]                  = "SM_STATE_OPERATIONAL",
};
#endif

////////////////////////////////////////////////////////////////////////////////
////////////////////////////// MESSAGE STATES //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

typedef enum
{
    SM_MESSAGE_STATE_IDLE,
    SM_MESSAGE_STATE_CFG_READ_NEXT,
    SM_MESSAGE_STATE_CFG_WRITE_NEXT,
    SM_MESSAGE_STATE_CFG_WRITE_ERROR,
    SM_MESSAGE_STATE_GPS_WRITE_NEXT,
    SM_MESSAGE_STATE_GPS_READ_NEXT,
    SM_MESSAGE_STATE_BLE_WRITE_NEXT,
    SM_MESSAGE_STATE_BLE_READ_NEXT,
    SM_MESSAGE_STATE_LOG_READ_NEXT,
    SM_MESSAGE_STATE_FW_SEND_IMAGE_NEXT,
} sm_message_state_t;
static sm_message_state_t message_state = SM_MESSAGE_STATE_IDLE;

// State specific context, used for maintaining information between config_if message sub-states
typedef struct
{
    union
    {
        struct
        {
            uint32_t length;
            uint8_t error_code;
        } cfg_write;

        struct
        {
            uint8_t * buffer_base;
            uint32_t  length;
            uint32_t  buffer_offset;
            uint16_t  last_index;
        } cfg_read;

        struct
        {
            uint8_t   address;
            uint16_t  length;
        } ble_write;

        struct
        {
            uint8_t   address;
            uint16_t  length;
        } ble_read;

        struct
        {
            uint32_t  length;
        } gps_write;

        struct
        {
            uint32_t  length;
        } gps_read;

        struct
        {
            uint32_t length;
            uint32_t start_offset;
            fs_handle_t file_handle;
        } log_read;

        struct
        {
            uint8_t image_type;
            uint32_t length;
            uint32_t crc32_supplied;
            uint32_t crc32_calculated;
            fs_handle_t file_handle;
        } fw_send_image;
    };
} sm_context_t;
static sm_context_t sm_context;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// GLOBALS /////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#define FS_FILE_ID_CONF             (0) // The File ID of the configuration data
#define FS_FILE_ID_STM32_IMAGE      (1) // STM32 application image
#define FS_FILE_ID_BLE_APP_IMAGE    (2) // BLE application image
#define FS_FILE_ID_BLE_SOFT_IMAGE   (3) // BLE soft-device image
#define FS_FILE_ID_LOG              (4) // Sensor log file

// Size of logging buffer that is used to store sensor data before it it written to FLASH
#define LOGGING_BUFFER_SIZE (256)

static volatile bool     config_if_tx_pending = false;
static volatile bool     config_if_rx_queued = false;
static bool              syshal_gps_bridging = false;
static bool              syshal_ble_bridging = false;
static volatile buffer_t config_if_send_buffer;
static volatile buffer_t config_if_receive_buffer;
static volatile buffer_t logging_buffer;
static volatile uint8_t  config_if_send_buffer_pool[SYSHAL_USB_PACKET_SIZE * 2];
static volatile uint8_t  config_if_receive_buffer_pool[SYSHAL_USB_PACKET_SIZE];
static volatile uint8_t  logging_buffer_pool[LOGGING_BUFFER_SIZE];
static uint8_t           spi_bridge_buffer[SYSHAL_USB_PACKET_SIZE + 1];
static uint32_t          config_if_message_timeout;
static volatile bool     config_if_connected = false;
static fs_t              file_system;
static volatile bool     gps_fix = false;
static fs_handle_t       log_file_handle = NULL;

////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// PROTOTYPES ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void config_if_timeout_reset(void);
static void message_set_state(sm_message_state_t s);

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// STARTUP ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void setup_buffers(void)
{
    // Send buffer
    buffer_init_policy(pool, &config_if_send_buffer,
                       (uintptr_t) &config_if_send_buffer_pool[0],
                       sizeof(config_if_send_buffer_pool), 2);

    // Receive buffer
    buffer_init_policy(pool, &config_if_receive_buffer,
                       (uintptr_t) &config_if_receive_buffer_pool[0],
                       sizeof(config_if_receive_buffer_pool), 1);

    // Logging buffer
    buffer_init_policy(pool, &logging_buffer,
                       (uintptr_t) &logging_buffer_pool[0],
                       sizeof(logging_buffer_pool), 1);
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// HELPER FUNCTIONS ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#define MIN(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

static void config_if_send_priv(volatile buffer_t * buffer)
{
    if (config_if_tx_pending)
        Throw(EXCEPTION_TX_BUSY);

    uintptr_t addr;
    uint32_t length = buffer_read(buffer, &addr);

    if (length)
    {
        config_if_tx_pending = true;
        config_if_send((uint8_t *) addr, length); // Send response
    }
    else
    {
        Throw(EXCEPTION_TX_BUFFER_FULL);
    }
}

static void config_if_receive_priv(void)
{
    if (!config_if_rx_queued)
    {
        // Queue receive
        uint8_t * receive_buffer;
        if (!buffer_write(&config_if_receive_buffer, (uintptr_t *)&receive_buffer))
            Throw(EXCEPTION_RX_BUFFER_FULL);

        if (CONFIG_IF_NO_ERROR == config_if_receive(receive_buffer, SYSHAL_USB_PACKET_SIZE))
            config_if_rx_queued = true;
    }
}

/**
 * @brief      Determines if any essential configuration tags are not set
 *
 * @return     True if essential configuration tags are not set
 */
static bool check_configuration_tags_set(void)
{
    // Check that all configuration tags are set
    bool tag_not_set = false;
    uint16_t tag, last_index = 0;
    int ret;
    bool ble_beacon_enabled;

    // Determine if Bluetooth beaconing is enabled
    ret = sys_config_get(SYS_CONFIG_TAG_BLUETOOTH_BEACON_ENABLE, NULL);
    if (ret < 0)
    {
        ble_beacon_enabled = false; // Tag is either not set or invalid, so default to disabled
    }
    else
    {
        ble_beacon_enabled = sys_config.sys_config_bluetooth_beacon_enable.contents.enable;
    }

    while (!sys_config_iterate(&tag, &last_index))
    {
        if (SYS_CONFIG_TAG_RTC_CURRENT_DATE_AND_TIME == tag)
            continue; // Skip the date and time as this is always set

        if (!ble_beacon_enabled) // If ble beacon is disabled
        {
            if (SYS_CONFIG_TAG_BLUETOOTH_BEACON_GEO_FENCE_TRIGGER_LOCATION == tag ||
                SYS_CONFIG_TAG_BLUETOOTH_BEACON_ADVERTISING_INTERVAL == tag ||
                SYS_CONFIG_TAG_BLUETOOTH_BEACON_ADVERTISING_CONFIGURATION == tag)
            {
                continue; // Then don't check the beacon tags
            }
        }

        void * src;
        ret = sys_config_get(tag, &src);

        if (SYS_CONFIG_ERROR_TAG_NOT_SET == ret)
        {
            tag_not_set = true;
        }
    }

    if (tag_not_set)
    {
        DEBUG_PR_WARN("Configuration tags not set");
    }

    return !tag_not_set;
}

void logging_add_to_buffer(uint8_t * data, uint32_t size)
{
    uint32_t length = 0;
    uint8_t * buf_ptr;
    if (!buffer_write(&logging_buffer, (uintptr_t *)&buf_ptr))
        Throw(EXCEPTION_TX_BUFFER_FULL);

    // Are we supposed to be adding a timestamp with this value?
    if (sys_config.sys_config_logging_date_time_stamp_enable.contents.enable)
    {
        logging_date_time_t * date_time = (logging_date_time_t *) buf_ptr;

        LOGGING_SET_HDR(date_time, LOGGING_DATE_TIME);

        syshal_rtc_data_and_time_t rtc_time;
        syshal_rtc_get_date_and_time(&rtc_time);

        date_time->year = rtc_time.year;
        date_time->month = rtc_time.month;
        date_time->day = rtc_time.day;
        date_time->hours = rtc_time.hours;
        date_time->minutes = rtc_time.minutes;
        date_time->seconds = rtc_time.seconds;

        buf_ptr += sizeof(logging_date_time_t);
        length += sizeof(logging_date_time_t);
    }

    if (sys_config.sys_config_logging_high_resolution_timer_enable.contents.enable)
    {
        DEBUG_PR_ERROR("logging_high_resolution_timer NOT IMPLEMENTED");
    }

    // Add the supplied data to the buffer
    memcpy(buf_ptr, data, size);
    length += size;

    buffer_write_advance(&logging_buffer, length);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////// CALLBACK FUNCTIONS //////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void syshal_axl_callback(syshal_axl_data_t data)
{
    DEBUG_PR_TRACE("%s() called", __FUNCTION__);

    // If accelerometer data logging is disabled
    if (!sys_config.sys_config_axl_log_enable.contents.enable)
    {
        syshal_axl_sleep(); // Sleep the accelerometer device
        return;
    }

    // FIXME: Check if we're in the correct state

    switch (sys_config.sys_config_axl_mode.contents.mode)
    {
        case SYS_CONFIG_AXL_MODE_PERIODIC:
            // FIXME: Log data!
            break;

        case SYS_CONFIG_AXL_MODE_TRIGGER_ABOVE:
            // Calculate vector magnitude
            __NOP(); // NOP required to give the switch case an instruction to jump to
            uint16_t magnitude_squared = (data.x * data.x) + (data.y * data.y) + (data.z * data.z); // WARN uint16_t maybe too small to contain true value
            // Determine if the read data is above the trigger point
            if (magnitude_squared >= sys_config.sys_config_axl_g_force_high_threshold.contents.threshold)
                __NOP(); // FIXME: Log data!
            break;
    }
}

void syshal_gps_callback(syshal_gps_event_t event)
{

    // If accelerometer data logging is disabled
    if (!sys_config.sys_config_gps_log_position_enable.contents.enable)
    {
        syshal_gps_shutdown(); // Sleep the gps device
        return;
    }

    switch (event.event_id)
    {
        case SYSHAL_GPS_EVENT_STATUS:
            DEBUG_PR_TRACE("SYSHAL_GPS_EVENT_STATUS - Fix: %u", event.event_data.status.gpsFix);
            if (event.event_data.status.gpsFix > 0)
                gps_fix = true;
            else
                gps_fix = false;
            break;
        case SYSHAL_GPS_EVENT_POSLLH:

            DEBUG_PR_TRACE("SYSHAL_GPS_EVENT_POSLLH - lat,long: %ld,%ld", event.event_data.location.lat, event.event_data.location.lon);

            // Add data to be logged
            if ( gps_fix && (SM_STATE_OPERATIONAL == sm_get_state()) )
            {
                logging_gps_position_t position;

                LOGGING_SET_HDR(&position, LOGGING_GPS_POSITION);
                position.iTOW = event.event_data.location.iTOW;
                position.lon = event.event_data.location.lon;
                position.lat = event.event_data.location.lat;
                position.height = event.event_data.location.height;

                logging_add_to_buffer((uint8_t *) &position, sizeof(position));
            }
            break;
        default:
            DEBUG_PR_WARN("Unknown GPS event in %s() : %d", __FUNCTION__, event.event_id);
            break;
    }

}

void syshal_switch_callback(syshal_switch_event_id_t event)
{
    switch (event)
    {
        case SYSHAL_SWITCH_EVENT_OPEN:
            if ((SM_STATE_OPERATIONAL == sm_get_state()))
            {
                logging_surfaced_t surfaced;
                LOGGING_SET_HDR(&surfaced, LOGGING_SURFACED);
                logging_add_to_buffer((uint8_t *) &surfaced, sizeof(surfaced));
            }
            break;
        case SYSHAL_SWITCH_EVENT_CLOSED:
            if ((SM_STATE_OPERATIONAL == sm_get_state()))
            {
                logging_submerged_t submerged;
                LOGGING_SET_HDR(&submerged, LOGGING_SUBMERGED);
                logging_add_to_buffer((uint8_t *) &submerged, sizeof(submerged));
            }
            break;

        default:
            DEBUG_PR_WARN("Unknown switch event in %s() : %d", __FUNCTION__, event);
            break;
    }
}

/**
 * @brief      Create the configuration file in FLASH memory
 *
 * @return     @ref FS_NO_ERROR on success.
 * @return     @ref FS_ERROR_FILE_ALREADY_EXISTS if the configuration file has
 *             already been created.
 * @return     @ref FS_ERROR_FLASH_MEDIA if the amount of data written to the file
 *             is not of expected length
 * @return     @ref FS_ERROR_NO_FREE_HANDLE if no file handle could be allocated.
 */
static int fs_create_configuration_data(void)
{
    // The configuration file does not exist so lets make one
    fs_handle_t file_system_handle;
    int ret = fs_open(file_system, &file_system_handle, FS_FILE_ID_CONF, FS_MODE_CREATE, NULL);

    if (FS_NO_ERROR != ret)
        return ret; // An unrecoverable error has occured

    fs_close(file_system_handle); // Close the newly created file and flush any data

    return FS_NO_ERROR;
}

/**
 * @brief      Deletes our configuration data file in FLASH
 *
 * @return     @ref FS_NO_ERROR on success.
 * @return     @ref FS_ERROR_FILE_NOT_FOUND if the configuration file was not
 *             found.
 * @return     @ref FS_ERROR_FILE_PROTECTED if the configuration file is write
 *             protected
 * @return     @ref FS_ERROR_NO_FREE_HANDLE if no file handle could be allocated.
 */
static int fs_delete_configuration_data(void)
{
    return (fs_delete(file_system, FS_FILE_ID_CONF));
}

/**
 * @brief      Write our configuration data from RAM to FLASH
 *
 * @return     @ref FS_NO_ERROR on success.
 * @return     @ref FS_ERROR_FILE_NOT_FOUND if the configuration file was not
 *             found.
 * @return     @ref FS_ERROR_FILE_PROTECTED if the configuration file is write
 *             protected
 * @return     @ref FS_ERROR_FLASH_MEDIA if the amount of data written to the file
 *             is not of expected length
 * @return     @ref FS_ERROR_NO_FREE_HANDLE if no file handle could be allocated.
 */
static int fs_set_configuration_data(void)
{
    fs_handle_t file_system_handle;
    int ret = fs_open(file_system, &file_system_handle, FS_FILE_ID_CONF, FS_MODE_WRITEONLY, NULL);

    if (FS_NO_ERROR != ret)
        return ret; // An unrecoverable error has occured

    // We have now created and opened our unformatted configuration file
    // So lets write our configuration data to it
    uint32_t bytes_written;
    ret = fs_write(file_system_handle, &sys_config, sizeof(sys_config), &bytes_written);

    fs_close(file_system_handle); // Close the file and flush any data

    if (FS_NO_ERROR != ret)
        return ret; // An unrecoverable error has occured

    if (bytes_written != sizeof(sys_config))
    {
        DEBUG_PR_WARN("%s() size mismatch", __FUNCTION__);
        return FS_ERROR_FLASH_MEDIA;
    }

    return FS_NO_ERROR;
}

/**
 * @brief      Load the configuration data from FLASH.
 *
 * @return     @ref FS_NO_ERROR on success.
 * @return     @ref FS_ERROR_FILE_NOT_FOUND if the configuration file was not
 *             found.
 * @return     @ref FS_ERROR_FLASH_MEDIA if the amount of data written to the file
 *             is not of expected length
 * @return     @ref FS_ERROR_NO_FREE_HANDLE if no file handle could be allocated.
 */
static int fs_get_configuration_data(void)
{
    fs_handle_t file_system_handle;
    // Attempt to open the configuration file
    int ret = fs_open(file_system, &file_system_handle, FS_FILE_ID_CONF, FS_MODE_READONLY, NULL);

    if (FS_NO_ERROR != ret)
        return ret; // An unrecoverable error has occured

    // We have opened the configuration file, so now we need to load the data from it
    uint32_t bytes_read;
    ret = fs_read(file_system_handle, &sys_config, sizeof(sys_config), &bytes_read);

    fs_close(file_system_handle); // We're done reading from this file

    if (FS_NO_ERROR != ret)
        return ret; // An unrecoverable error has occured

    if (bytes_read != sizeof(sys_config))
    {
        DEBUG_PR_WARN("%s() size mismatch", __FUNCTION__);
        return FS_ERROR_FLASH_MEDIA;
    }

    return FS_NO_ERROR;
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// CFG_READ ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void cfg_read_populate_next(uint16_t tag, void * src, uint16_t length)
{
    sm_context.cfg_read.buffer_base[sm_context.cfg_read.buffer_offset++] = tag & 0xFF;
    sm_context.cfg_read.buffer_base[sm_context.cfg_read.buffer_offset++] = (tag >> 8) & 0xFF;
    memcpy(&sm_context.cfg_read.buffer_base[sm_context.cfg_read.buffer_offset],
           src, length);
    sm_context.cfg_read.buffer_offset += length;
}

void cfg_read_populate_buffer(void)
{
    uint16_t tag;
    int ret;

    /* Iterate configuration tags */
    while (!sys_config_iterate(&tag, &sm_context.cfg_read.last_index))
    {
        void * src;
        ret = sys_config_get(tag, &src);
        if (ret > 0)
        {
            if ((sm_context.cfg_read.buffer_offset + ret + sizeof(uint16_t)) > SYSHAL_USB_PACKET_SIZE)
            {
                /* Buffer is full so defer this to the next iteration */
                sm_context.cfg_read.last_index--;
                break;
            }

            cfg_read_populate_next(tag, src, (uint16_t)ret);
        }
    }
}

uint32_t cfg_read_all_calc_length(void)
{
    int ret;
    uint16_t last_index = 0, tag;
    uint32_t length = 0;

    /* Iterate all configuration tags */
    while (!sys_config_iterate(&tag, &last_index))
    {
        void * src;
        ret = sys_config_get(tag, &src);
        if (ret > 0)
            length += (ret + sizeof(uint16_t));
    }

    return length;
}

void cfg_read_req(cmd_t * req, uint16_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_cfg_read_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);

    CMD_SET_HDR(resp, CMD_CFG_READ_RESP);

    /* Allocate buffer for following configuration data */
    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_cfg_read_resp_t));
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&sm_context.cfg_read.buffer_base))
        Throw(EXCEPTION_TX_BUFFER_FULL);

    /* Reset buffer offset to head of buffer */
    sm_context.cfg_read.buffer_offset = 0;

    if (CFG_READ_REQ_READ_ALL == req->p.cmd_cfg_read_req.configuration_tag)
    {
        /* Requested all configuration items */
        resp->p.cmd_cfg_read_resp.error_code = CMD_NO_ERROR;
        resp->p.cmd_cfg_read_resp.length = cfg_read_all_calc_length();
        sm_context.cfg_read.last_index = 0;
        sm_context.cfg_read.length = resp->p.cmd_cfg_read_resp.length;
        if (resp->p.cmd_cfg_read_resp.length > 0)
        {
            cfg_read_populate_buffer();
            buffer_write_advance(&config_if_send_buffer, sm_context.cfg_read.buffer_offset);
        }
    }
    else
    {
        void * src;
        /* Requested a single configuration tag */
        int ret = sys_config_get(req->p.cmd_cfg_read_req.configuration_tag, &src);

        if (ret < 0)
        {
            resp->p.cmd_cfg_read_resp.length = 0;
            if (SYS_CONFIG_ERROR_INVALID_TAG == ret)  // Tag is not valid. Return an error code
            {
                resp->p.cmd_cfg_read_resp.error_code = CMD_ERROR_INVALID_CONFIG_TAG;
            }
            else if (SYS_CONFIG_ERROR_TAG_NOT_SET == ret)  // Tag is not set. Return an error code
            {
                resp->p.cmd_cfg_read_resp.error_code = CMD_ERROR_CONFIG_TAG_NOT_SET;
            }
            else
            {
                Throw(EXCEPTION_BAD_SYS_CONFIG_ERROR_CONDITION);
            }
        }
        else
        {
            cfg_read_populate_next(req->p.cmd_cfg_read_req.configuration_tag, src, (uint16_t)ret);
            resp->p.cmd_cfg_read_resp.error_code = CMD_NO_ERROR;
            resp->p.cmd_cfg_read_resp.length = sm_context.cfg_read.buffer_offset;
            sm_context.cfg_read.length = sm_context.cfg_read.buffer_offset;
            buffer_write_advance(&config_if_send_buffer, sm_context.cfg_read.buffer_offset);
        }
    }

    config_if_send_priv(&config_if_send_buffer); // Send the response

    if (resp->p.cmd_cfg_read_resp.length > 0)
    {
        /* Another buffer must follow the initial response */
        message_set_state(SM_MESSAGE_STATE_CFG_READ_NEXT);
    }
}

void cfg_read_next_state(void)
{
    /* Send the pending buffer and prepare a new buffer */
    config_if_send_priv(&config_if_send_buffer);
    sm_context.cfg_read.length -= sm_context.cfg_read.buffer_offset;

    if (sm_context.cfg_read.length > 0)
    {
        /* Allocate buffer for following configuration data */
        if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&sm_context.cfg_read.buffer_base))
            Throw(EXCEPTION_TX_BUFFER_FULL);

        /* Reset buffer offset to head of buffer */
        sm_context.cfg_read.buffer_offset = 0;
        cfg_read_populate_buffer();

        /* Advance the buffer */
        buffer_write_advance(&config_if_send_buffer, sm_context.cfg_read.buffer_offset);
    }
    else
    {
        message_set_state(SM_MESSAGE_STATE_IDLE);
    }
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// CFG_WRITE //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void cfg_write_req(cmd_t * req, uint16_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_cfg_write_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    sm_context.cfg_write.length = req->p.cmd_cfg_write_req.length;

    // Length is zero
    if (!sm_context.cfg_write.length)
        Throw(EXCEPTION_PACKET_WRONG_SIZE);

    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);
    resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer); // Send response

    config_if_receive_priv(); // Queue a receive

    message_set_state(SM_MESSAGE_STATE_CFG_WRITE_NEXT);
}

static void cfg_write_next_state(void)
{
    uint8_t * read_buffer;
    uint32_t length = buffer_read(&config_if_receive_buffer, (uintptr_t *)&read_buffer);

    if (!length)
        return;

    buffer_read_advance(&config_if_receive_buffer, length); // Remove this packet from the receive buffer

    if (length > sm_context.cfg_write.length)
    {
        // If it is not we should exit out and return an error
        sm_context.cfg_write.error_code = CMD_ERROR_DATA_OVERSIZE;
        message_set_state(SM_MESSAGE_STATE_CFG_WRITE_ERROR);
        Throw(EXCEPTION_PACKET_WRONG_SIZE);
    }

    // Lets deconstruct this packet of configuration data/ID pairs
    uint32_t buffer_offset = 0;

    while (buffer_offset < length)
    {
        // Extract the tag ID
        uint16_t tag = 0;
        tag |= (uint16_t) read_buffer[buffer_offset++] & 0x00FF;
        tag |= (uint16_t) (read_buffer[buffer_offset++] << 8) & 0xFF00;

        int tag_size = sys_config_size(tag);

        // Check tag is valid
        if (tag_size < 0)
        {
            // If it is not we should exit out and return an error
            sm_context.cfg_write.error_code = CMD_ERROR_INVALID_CONFIG_TAG;
            message_set_state(SM_MESSAGE_STATE_CFG_WRITE_ERROR);
            Throw(EXCEPTION_BAD_SYS_CONFIG_ERROR_CONDITION);
        }

        int ret = sys_config_set(tag, &read_buffer[buffer_offset], tag_size); // Set tag value

        if (ret < 0)
        {
            DEBUG_PR_TRACE("sys_config_set() returned: %d()", ret);
            message_set_state(SM_MESSAGE_STATE_IDLE);
            Throw(EXCEPTION_BAD_SYS_CONFIG_ERROR_CONDITION); // Exit and fail silent
        }

        // Have we just changed our GPS baudrate
        if (SYS_CONFIG_TAG_GPS_UART_BAUD_RATE == tag)
        {
            // If so update our UART HW baudrate
            syshal_uart_change_baud(GPS_UART, sys_config.sys_config_gps_uart_baud_rate.contents.baudrate);
        }

        buffer_offset += tag_size;
    }

    sm_context.cfg_write.length -= buffer_offset;

    if (sm_context.cfg_write.length) // Is there still data to receive?
    {
        config_if_receive_priv(); // Queue a receive
        config_if_timeout_reset(); // Reset the message timeout counter
    }
    else
    {
        // We have received all the data
        // Then send a confirmation
        cmd_t * resp;
        if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
            Throw(EXCEPTION_TX_BUFFER_FULL);
        CMD_SET_HDR(resp, CMD_CFG_WRITE_CNF);
        resp->p.cmd_cfg_write_cnf.error_code = CMD_NO_ERROR;

        buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_cfg_write_cnf_t));
        config_if_send_priv(&config_if_send_buffer); // Send response

        message_set_state(SM_MESSAGE_STATE_IDLE);
    }

}

static void cfg_write_error_state(void)
{
    // Return an error code
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_CFG_WRITE_CNF);
    resp->p.cmd_cfg_write_cnf.error_code = sm_context.cfg_write.error_code;

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_cfg_write_cnf_t));
    config_if_send_priv(&config_if_send_buffer); // Send response

    message_set_state(SM_MESSAGE_STATE_IDLE);
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// CFG_SAVE ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void cfg_save_req(cmd_t * req, uint16_t size)
{
    // Check request size is correct
    if (size != CMD_SIZE_HDR)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    int ret = fs_delete_configuration_data(); // Must first delete our configuration data

    switch (ret)
    {
        case FS_ERROR_FILE_NOT_FOUND: // If there is no configuration file, then make one
        case FS_NO_ERROR:

            ret = fs_create_configuration_data(); // Re/Create the file
            if (FS_NO_ERROR != ret)
                Throw(EXCEPTION_FS_ERROR);

            ret = fs_set_configuration_data(); // Flush our RAM configuration data to the file
            if (FS_NO_ERROR != ret)
                Throw(EXCEPTION_FS_ERROR);

            resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
            break;

        case FS_ERROR_FILE_PROTECTED:
            resp->p.cmd_generic_resp.error_code = CMD_ERROR_CONFIG_PROTECTED;
            break;

        default:
            Throw(EXCEPTION_FS_ERROR);
            break;
    }

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer); // Send confirmation
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// CFG_RESTORE /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void cfg_restore_req(cmd_t * req, uint16_t size)
{
    // Check request size is correct
    if (size != CMD_SIZE_HDR)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    int ret = fs_get_configuration_data();

    switch (ret)
    {
        case FS_NO_ERROR:
            resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
            break;

        case FS_ERROR_FILE_NOT_FOUND:
            resp->p.cmd_generic_resp.error_code = CMD_ERROR_FILE_NOT_FOUND;
            break;

        default:
            Throw(EXCEPTION_FS_ERROR);
            break;
    }

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer); // Send confirmation
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// CFG_ERASE //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void cfg_erase_req(cmd_t * req, uint16_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_cfg_erase_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    if (CFG_ERASE_REQ_ERASE_ALL == req->p.cmd_cfg_erase_req.configuration_tag) // Erase all configuration tags
    {
        uint16_t last_index = 0;
        uint16_t tag;

        while (!sys_config_iterate(&tag, &last_index))
        {
            sys_config_unset(tag);
        }

        resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
    }
    else
    {
        // Erase just one configuration tag
        int return_code = sys_config_unset(req->p.cmd_cfg_erase_req.configuration_tag);

        switch (return_code)
        {
            case SYS_CONFIG_NO_ERROR:
                resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
                break;

            case SYS_CONFIG_ERROR_INVALID_TAG:
                resp->p.cmd_generic_resp.error_code = CMD_ERROR_INVALID_CONFIG_TAG;
                break;

            default:
                Throw(EXCEPTION_FS_ERROR);
                break;
        }
    }

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer); // Send confirmation
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// CFG_PROTECT //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void cfg_protect_req(cmd_t * req, uint16_t size)
{
    // Check request size is correct
    if (size != CMD_SIZE_HDR)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    int ret = fs_protect(file_system, FS_FILE_ID_CONF);

    switch (ret)
    {
        case FS_NO_ERROR:
            resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
            break;

        case FS_ERROR_FILE_NOT_FOUND:
            resp->p.cmd_generic_resp.error_code = CMD_ERROR_FILE_NOT_FOUND;
            break;

        default:
            Throw(EXCEPTION_FS_ERROR);
            break;
    }

    // Send the response
    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// CFG_UNPROTECT /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void cfg_unprotect_req(cmd_t * req, uint16_t size)
{
    // Check request size is correct
    if (size != CMD_SIZE_HDR)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    int ret = fs_unprotect(file_system, FS_FILE_ID_CONF);

    switch (ret)
    {
        case FS_NO_ERROR:
            resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
            break;

        case FS_ERROR_FILE_NOT_FOUND:
            resp->p.cmd_generic_resp.error_code = CMD_ERROR_FILE_NOT_FOUND;
            break;

        default:
            Throw(EXCEPTION_FS_ERROR);
            break;
    }

    // Send the response
    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// GPS_WRITE ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void gps_write_req(cmd_t * req, uint16_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_gps_write_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    // Generate and send response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    // If bridging is enabled
    if (syshal_gps_bridging)
    {
        sm_context.gps_write.length = req->p.cmd_gps_write_req.length;
        resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;

        config_if_receive_priv(); // Queue a receive

        message_set_state(SM_MESSAGE_STATE_GPS_WRITE_NEXT);
    }
    else
    {
        // Bridging is not enabled so return an error code
        resp->p.cmd_generic_resp.error_code = CMD_ERROR_BRIDGING_DISABLED;
    }

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

static void gps_write_next_state(void)
{
    uint8_t * read_buffer;
    uint32_t length = buffer_read(&config_if_receive_buffer, (uintptr_t *)&read_buffer);

    if (!length)
        return;

    buffer_read_advance(&config_if_receive_buffer, length); // Remove this packet from the receive buffer

    if (length > sm_context.gps_write.length)
    {
        message_set_state(SM_MESSAGE_STATE_IDLE);
        Throw(EXCEPTION_PACKET_WRONG_SIZE);
    }

    int ret = syshal_gps_send_raw(read_buffer, length);

    // Check send worked
    if (ret < 0)
    {
        // If not we should exit out
        message_set_state(SM_MESSAGE_STATE_IDLE);
        Throw(EXCEPTION_GPS_SEND_ERROR);
    }

    sm_context.gps_write.length -= length;

    if (sm_context.gps_write.length) // Is there still data to receive?
    {
        config_if_receive_priv(); // Queue a receive
        config_if_timeout_reset(); // Reset the message timeout counter
    }
    else
    {
        // We have received all the data
        message_set_state(SM_MESSAGE_STATE_IDLE);
    }
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// GPS_READ ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void gps_read_req(cmd_t * req, uint16_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_gps_read_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    // Generate and send response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GPS_READ_RESP);

    // If bridging is enabled
    if (syshal_gps_bridging)
    {
        // Try to match the requested length, if not return as close to it as we can
        //DEBUG_PR_TRACE("syshal_gps_available_raw() = %lu, req->p.cmd_gps_read_req.length = %lu", syshal_gps_available_raw(), req->p.cmd_gps_read_req.length);
        sm_context.gps_read.length = MIN(syshal_gps_available_raw(), req->p.cmd_gps_read_req.length);

        resp->p.cmd_gps_read_resp.length = sm_context.gps_read.length;
        resp->p.cmd_gps_read_resp.error_code = CMD_NO_ERROR;
    }
    else
    {
        // Bridging is not enabled so return an error code
        resp->p.cmd_gps_read_resp.length = 0;
        resp->p.cmd_gps_read_resp.error_code = CMD_ERROR_BRIDGING_DISABLED;
    }

    // Send response
    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_gps_read_resp_t));
    config_if_send_priv(&config_if_send_buffer);

    if (sm_context.gps_read.length > 0)
        message_set_state(SM_MESSAGE_STATE_GPS_READ_NEXT);
}

static void gps_read_next_state(void)
{
    // Generate and send response
    uint8_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);

    // Don't read more than the maximum packet size
    uint32_t bytes_to_read = MIN(sm_context.gps_read.length, SYSHAL_USB_PACKET_SIZE);

    // Receive data from the GPS module
    uint32_t bytes_actually_read = syshal_gps_receive_raw(resp, bytes_to_read);

    sm_context.gps_read.length -= bytes_actually_read;

    // Send response
    buffer_write_advance(&config_if_send_buffer, bytes_actually_read);
    config_if_send_priv(&config_if_send_buffer);

    if (sm_context.gps_read.length) // Is there still data to send?
    {
        config_if_timeout_reset(); // Reset the message timeout counter
    }
    else
    {
        // We have sent all the data
        message_set_state(SM_MESSAGE_STATE_IDLE);
    }
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// GPS_CONFIG_REQ ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void gps_config_req(cmd_t * req, uint16_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_gps_config_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    syshal_gps_bridging = req->p.cmd_gps_config_req.enable; // Disable or enable GPS bridging

    // If we've just enabled bridging, remove any previous data in the GPS rx buffer
    if (syshal_gps_bridging)
    {
        uint8_t flush;
        while (syshal_gps_receive_raw(&flush, 1))
        {}
    }

    // Generate and send response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

static void ble_config_req(cmd_t * req, uint16_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_ble_config_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    syshal_ble_bridging = req->p.cmd_ble_config_req.enable; // Disable or enable BLE bridging

    // Generate and send response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

static void ble_write_req(cmd_t * req, uint16_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_ble_write_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    // Generate and send response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    // If bridging is enabled
    if (syshal_ble_bridging)
    {
        sm_context.ble_write.address = req->p.cmd_ble_write_req.address;
        sm_context.ble_write.length = req->p.cmd_ble_write_req.length;
        resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;

        config_if_receive_priv(); // Queue a receive

        message_set_state(SM_MESSAGE_STATE_BLE_WRITE_NEXT);
    }
    else
    {
        // Bridging is not enabled so return an error code
        resp->p.cmd_generic_resp.error_code = CMD_ERROR_BRIDGING_DISABLED;
    }

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

static void ble_write_next_state(void)
{
    uint8_t * read_buffer;
    uint32_t length = buffer_read(&config_if_receive_buffer, (uintptr_t *)&read_buffer);

    if (!length)
        return;

    buffer_read_advance(&config_if_receive_buffer, length); // Remove this packet from the receive buffer

    if (length > sm_context.ble_write.length)
    {
        message_set_state(SM_MESSAGE_STATE_IDLE);
        Throw(EXCEPTION_PACKET_WRONG_SIZE);
    }

    spi_bridge_buffer[0] = sm_context.ble_write.address;
    memcpy(&spi_bridge_buffer[1], read_buffer, length);

    // Check send worked
    if (syshal_spi_transfer(SPI_BLE, spi_bridge_buffer, NULL, length + 1))
    {
        // If not we should exit out
        message_set_state(SM_MESSAGE_STATE_IDLE);
        Throw(EXCEPTION_SPI_ERROR);
    }

    sm_context.ble_write.length -= length;

    if (sm_context.ble_write.length) // Is there still data to receive?
    {
        config_if_receive_priv(); // Queue a receive
        config_if_timeout_reset(); // Reset the message timeout counter
    }
    else
    {
        // We have received all the data
        message_set_state(SM_MESSAGE_STATE_IDLE);
    }
}

static void ble_read_req(cmd_t * req, uint16_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_ble_read_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    // Generate and send response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    // If bridging is enabled
    if (syshal_ble_bridging)
    {
        sm_context.ble_read.address = req->p.cmd_ble_read_req.address;
        sm_context.ble_read.length = req->p.cmd_ble_read_req.length;
        resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
    }
    else
    {
        // Bridging is not enabled so return an error code
        sm_context.ble_read.length = 0;
        resp->p.cmd_generic_resp.error_code = CMD_ERROR_BRIDGING_DISABLED;
    }

    // Send response
    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer);

    if (sm_context.ble_read.length > 0)
        message_set_state(SM_MESSAGE_STATE_BLE_READ_NEXT);
}

static void ble_read_next_state(void)
{
    // Generate and send response
    uint8_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);

    // Don't read more than the maximum packet size less one byte which shall
    // be used for storing the SPI bus address
    uint32_t bytes_to_read = MIN(sm_context.ble_read.length, SYSHAL_USB_PACKET_SIZE);

    // Read data from the specified SPI bus address
    memset(spi_bridge_buffer, 0, sizeof(spi_bridge_buffer));
    spi_bridge_buffer[0] = sm_context.ble_read.address;
    if (syshal_spi_transfer(SPI_BLE, spi_bridge_buffer, spi_bridge_buffer, bytes_to_read + 1))
    {
        // If not we should exit out
        message_set_state(SM_MESSAGE_STATE_IDLE);
        Throw(EXCEPTION_SPI_ERROR);
    }

    sm_context.ble_read.length -= bytes_to_read;

    // Copy data from bridge buffer into USB buffer
    memcpy(resp, &spi_bridge_buffer[1], bytes_to_read);

    // Send response
    buffer_write_advance(&config_if_send_buffer, bytes_to_read);
    config_if_send_priv(&config_if_send_buffer);

    if (sm_context.ble_read.length) // Is there still data to send?
    {
        config_if_timeout_reset(); // Reset the message timeout counter
    }
    else
    {
        // We have sent all the data
        message_set_state(SM_MESSAGE_STATE_IDLE);
    }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// STATUS_REQ //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void status_req(cmd_t * req, uint16_t size)
{
    // Check request size is correct
    if (size != CMD_SIZE_HDR)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    // Generate and send response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_STATUS_RESP);

    DEBUG_PR_WARN("%s() NOT IMPLEMENTED, responding with spoof data", __FUNCTION__);

    resp->p.cmd_status_resp.error_code = CMD_NO_ERROR;
    resp->p.cmd_status_resp.stm_firmware_version = 0;
    resp->p.cmd_status_resp.ble_firmware_version = 0;
    resp->p.cmd_status_resp.configuration_format_version = 0;

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_status_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

static void fw_send_image_req(cmd_t * req, uint16_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_fw_send_image_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    // Generate and send response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    // Store variables for use in future
    sm_context.fw_send_image.length = req->p.cmd_fw_send_image_req.length;
    sm_context.fw_send_image.crc32_supplied = req->p.cmd_fw_send_image_req.CRC32;
    sm_context.fw_send_image.crc32_calculated = 0;

    // Check the image type is correct
    sm_context.fw_send_image.image_type = req->p.cmd_fw_send_image_req.image_type;

    if ( (sm_context.fw_send_image.image_type == FS_FILE_ID_STM32_IMAGE)
         || (sm_context.fw_send_image.image_type == FS_FILE_ID_BLE_SOFT_IMAGE)
         || (sm_context.fw_send_image.image_type == FS_FILE_ID_BLE_APP_IMAGE))
    {
        int ret = fs_delete(file_system, sm_context.fw_send_image.image_type); // Must first delete any current image

        switch (ret)
        {
            case FS_ERROR_FILE_NOT_FOUND: // If there is no image file, then make one
            case FS_NO_ERROR:
                __NOP(); // Instruct for switch case to jump to
                int ret = fs_open(file_system, &sm_context.fw_send_image.file_handle, sm_context.fw_send_image.image_type, FS_MODE_CREATE, NULL);

                if (FS_NO_ERROR != ret)
                    Throw(EXCEPTION_FS_ERROR); // An unrecoverable error has occured

                // FIXME: Check to see if there is sufficient room for the firmware image
                resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
                message_set_state(SM_MESSAGE_STATE_FW_SEND_IMAGE_NEXT);
                break;

            case FS_ERROR_FILE_PROTECTED: // We never lock the fw images so this shouldn't occur
                resp->p.cmd_generic_resp.error_code = CMD_ERROR_CONFIG_PROTECTED;
                break;

            default:
                Throw(EXCEPTION_FS_ERROR);
                break;
        }
    }
    else
    {
        resp->p.cmd_generic_resp.error_code = CMD_ERROR_INVALID_FW_IMAGE_TYPE;
    }

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

static void fw_send_image_next_state(void)
{
    uint8_t * read_buffer;
    uint32_t length = buffer_read(&config_if_receive_buffer, (uintptr_t *)&read_buffer);

    if (!length)
        return;

    buffer_read_advance(&config_if_receive_buffer, length); // Remove this packet from the receive buffer

    // Is this packet larger than we were expecting?
    if (length > sm_context.fw_send_image.length)
    {
        // If it is we should exit out
        message_set_state(SM_MESSAGE_STATE_IDLE);
        fs_close(sm_context.fw_send_image.file_handle);
        fs_delete(file_system, sm_context.fw_send_image.image_type);
        Throw(EXCEPTION_PACKET_WRONG_SIZE);
    }

    sm_context.fw_send_image.crc32_calculated = crc32(sm_context.fw_send_image.crc32_calculated, read_buffer, length);
    uint32_t bytes_written = 0;
    int ret = fs_write(sm_context.fw_send_image.file_handle, read_buffer, length, &bytes_written);
    if (FS_NO_ERROR != ret)
    {
        fs_close(sm_context.fw_send_image.file_handle);
        fs_delete(file_system, sm_context.fw_send_image.image_type);
        message_set_state(SM_MESSAGE_STATE_IDLE);
        Throw(EXCEPTION_FS_ERROR);
    }

    sm_context.fw_send_image.length -= length;

    if (sm_context.fw_send_image.length) // Is there still data to receive?
    {
        config_if_receive_priv(); // Queue a receive
        config_if_timeout_reset(); // Reset the message timeout counter
    }
    else
    {
        // We have received all the data
        fs_close(sm_context.fw_send_image.file_handle); // Close the file

        // Then send a confirmation
        cmd_t * resp;
        if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
            Throw(EXCEPTION_TX_BUFFER_FULL);
        CMD_SET_HDR(resp, CMD_FW_SEND_IMAGE_COMPLETE_CNF);

        // Check the CRC32 is correct
        if (sm_context.fw_send_image.crc32_calculated == sm_context.fw_send_image.crc32_supplied)
        {
            resp->p.cmd_fw_send_image_complete_cnf.error_code = CMD_NO_ERROR;
        }
        else
        {
            resp->p.cmd_fw_send_image_complete_cnf.error_code = CMD_ERROR_IMAGE_CRC_MISMATCH;
            fs_delete(file_system, sm_context.fw_send_image.image_type); // Image is invalid, so delete it
        }

        buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_fw_send_image_complete_cnf_t));
        config_if_send_priv(&config_if_send_buffer); // Send response

        message_set_state(SM_MESSAGE_STATE_IDLE);
    }
}

static void fw_apply_image_req(cmd_t * req, uint16_t size)
{
    DEBUG_PR_WARN("%s NOT FULLY IMPLEMENTED", __FUNCTION__);

    fs_handle_t file_system_handle;

    // Check request size is correct
    if (CMD_SIZE(cmd_fw_apply_image_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    // Generate response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    // Check the image type is correct
    uint8_t image_type = req->p.cmd_fw_apply_image_req.image_type;

    if ( (image_type == FS_FILE_ID_STM32_IMAGE)
         || (image_type == FS_FILE_ID_BLE_SOFT_IMAGE)
         || (image_type == FS_FILE_ID_BLE_APP_IMAGE))
    {
        // Check image exists
        int ret = fs_open(file_system, &file_system_handle, image_type, FS_MODE_READONLY, NULL);

        switch (ret)
        {
            case FS_NO_ERROR:
                resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
                break;

            case FS_ERROR_FILE_NOT_FOUND:
                resp->p.cmd_generic_resp.error_code = CMD_ERROR_FILE_NOT_FOUND;
                break;

            default:
                Throw(EXCEPTION_FS_ERROR);
                break;
        }
    }
    else
    {
        resp->p.cmd_generic_resp.error_code = CMD_ERROR_INVALID_FW_IMAGE_TYPE;
    }

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer);

    if (CMD_NO_ERROR == resp->p.cmd_generic_resp.error_code)
    {
        // There was no error, so wait for the confirmation to be sent
#ifndef GTEST // Prevent infinite loop in gtest
        while (config_if_tx_pending)
        {}
#endif

        // And then apply the firmware image
        switch (image_type)
        {
            case FS_FILE_ID_STM32_IMAGE:
                // FIXME: needs implementing
                break;

            case FS_FILE_ID_BLE_SOFT_IMAGE:
                // FIXME: needs implementing
                break;

            case FS_FILE_ID_BLE_APP_IMAGE:
                // FIXME: needs implementing
                break;
        }

        fs_close(file_system_handle); // Close the file
    }
}

static void reset_req(cmd_t * req, uint16_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_reset_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    // Generate and send response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer);

    // Wait for response to have been sent
    while (config_if_tx_pending)
    {}

    syshal_pmu_reset();

    // If a system reset isn't available then block until a watchdog reset
    for (;;) {}
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////// BATTERY_STATUS_REQ //////////////////////////////
////////////////////////////////////////////////////////////////////////////////
static void battery_status_req(cmd_t * req, uint16_t size)
{
    // Check request size is correct
    if (size != CMD_SIZE_HDR)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    // Generate and send response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_BATTERY_STATUS_RESP);

#ifdef DUMMY_BATTERY_MONITOR
    DEBUG_PR_WARN("%s() NOT IMPLEMENTED, responding with spoof data", __FUNCTION__);

    resp->p.cmd_battery_status_resp.error_code = CMD_NO_ERROR;
    resp->p.cmd_battery_status_resp.charging_indicator = 1;
    resp->p.cmd_battery_status_resp.charge_level = 100;
#else
    resp->p.cmd_battery_status_resp.error_code = CMD_NO_ERROR;
    resp->p.cmd_battery_status_resp.charging_indicator = syshal_batt_charging();
    resp->p.cmd_battery_status_resp.charge_level = syshal_batt_level();
#endif

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_battery_status_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// LOG_CREATE_REQ ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void log_create_req(cmd_t * req, uint16_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_log_create_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    // Generate and send response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    // Get request's parameters
    uint8_t mode = req->p.cmd_log_create_req.mode;
    uint8_t sync_enable = req->p.cmd_log_create_req.sync_enable;
    uint8_t max_file_size = req->p.cmd_log_create_req.max_file_size;

    UNUSED(max_file_size);

    // Attempt to create the log file
    if (CMD_LOG_CREATE_REQ_MODE_FILL == mode || CMD_LOG_CREATE_REQ_MODE_CIRCULAR == mode)
    {
        // Convert from create mode to fs_mode_t
        fs_mode_t fs_mode;
        if (CMD_LOG_CREATE_REQ_MODE_FILL == mode)
            fs_mode = FS_MODE_CREATE;
        else if (CMD_LOG_CREATE_REQ_MODE_CIRCULAR == mode)
            fs_mode = FS_MODE_CREATE_CIRCULAR;

        fs_handle_t file_system_handle;
        int ret = fs_open(file_system, &file_system_handle, FS_FILE_ID_LOG, fs_mode, &sync_enable);

        switch (ret)
        {
            case FS_NO_ERROR:
                resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
                fs_close(file_system_handle); // Close the file

                // Set the sys_config log file type
                sys_config_logging_file_type_t log_file_type;
                log_file_type.contents.file_type = mode;
                sys_config_set(SYS_CONFIG_TAG_LOGGING_FILE_TYPE, &log_file_type.contents, SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_file_type_t));

                // Set the sys_config log file size
                sys_config_logging_file_size_t log_file_size;
                log_file_size.contents.file_size = mode;
                sys_config_set(SYS_CONFIG_TAG_LOGGING_FILE_SIZE, &log_file_size.contents, SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_file_size_t));
                break;

            case FS_ERROR_FILE_ALREADY_EXISTS:
                resp->p.cmd_generic_resp.error_code = CMD_ERROR_FILE_ALREADY_EXISTS;
                break;

            default:
                Throw(EXCEPTION_FS_ERROR);
                break;
        }
    }
    else
    {
        resp->p.cmd_generic_resp.error_code = CMD_ERROR_INVALID_PARAMETER;
    }

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// LOG_ERASE_REQ /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void log_erase_req(cmd_t * req, uint16_t size)
{
    // Check request size is correct
    if (size != CMD_SIZE_HDR)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    // Generate response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    int ret = fs_delete(file_system, FS_FILE_ID_LOG);

    switch (ret)
    {
        case FS_NO_ERROR:
            resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
            break;

        case FS_ERROR_FILE_NOT_FOUND:
            resp->p.cmd_generic_resp.error_code = CMD_ERROR_FILE_NOT_FOUND;
            break;

        case FS_ERROR_FILE_PROTECTED:
            resp->p.cmd_generic_resp.error_code = CMD_ERROR_CONFIG_PROTECTED;
            break;

        default:
            Throw(EXCEPTION_FS_ERROR);
            break;
    }

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// LOG_READ_REQ /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void log_read_req(cmd_t * req, uint16_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_log_read_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    // Generate response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_LOG_READ_RESP);

    sm_context.log_read.length = 0;

    fs_stat_t stat;
    int ret = fs_stat(file_system, FS_FILE_ID_LOG, &stat);

    switch (ret)
    {
        case FS_NO_ERROR:
            sm_context.log_read.length = req->p.cmd_log_read_req.length;
            sm_context.log_read.start_offset = req->p.cmd_log_read_req.start_offset;

            // Check if both parameters are zero. If they are the client is requesting a full log file
            if ((0 == sm_context.log_read.length) && (0 == sm_context.log_read.start_offset))
                sm_context.log_read.length = stat.size;

            if (sm_context.log_read.start_offset > stat.size) // Is the offset beyond the end of the file?
            {
                resp->p.cmd_log_read_resp.error_code = CMD_ERROR_INVALID_PARAMETER;
            }
            else
            {
                // Do we have this amount of data ready to read?
                if ((sm_context.log_read.length + sm_context.log_read.start_offset) > stat.size)
                {
                    // If the length requested is greater than what we have then reduce the length
                    sm_context.log_read.length = stat.size - sm_context.log_read.start_offset;
                }

                // Open the file
                ret = fs_open(file_system, &sm_context.log_read.file_handle, FS_FILE_ID_LOG, FS_MODE_READONLY, NULL);

                if (FS_NO_ERROR == ret)
                {
                    resp->p.cmd_log_read_resp.error_code = CMD_NO_ERROR;
                    if (sm_context.log_read.length)
                        message_set_state(SM_MESSAGE_STATE_LOG_READ_NEXT);
                    else
                        fs_close(sm_context.log_read.file_handle);
                }
                else
                {
                    Throw(EXCEPTION_FS_ERROR);
                }
            }
            break;

        case FS_ERROR_FILE_NOT_FOUND:
            resp->p.cmd_log_read_resp.error_code = CMD_ERROR_FILE_NOT_FOUND;
            break;

        default:
            Throw(EXCEPTION_FS_ERROR);
            break;
    }

    resp->p.cmd_log_read_resp.length = sm_context.log_read.length;

    DEBUG_PR_TRACE("responding with error code: %d, and length %lu", resp->p.cmd_log_read_resp.error_code, sm_context.log_read.length);

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_log_read_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

static void log_read_next_state()
{
    uint32_t bytes_to_read;
    uint32_t bytes_actually_read;
    int ret;

    // Get write buffer
    uint8_t * read_buffer;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&read_buffer))
        Throw(EXCEPTION_TX_BUFFER_FULL);

    // Is there a reading offset?
    while (sm_context.log_read.start_offset)
    {
        // If so move to this location in packet sized chunks
        bytes_to_read = MIN(sm_context.log_read.start_offset, SYSHAL_USB_PACKET_SIZE);

        ret = fs_read(sm_context.log_read.file_handle, &read_buffer, bytes_to_read, &bytes_actually_read);
        if (FS_NO_ERROR != ret)
        {
            Throw(EXCEPTION_FS_ERROR);
        }

        sm_context.log_read.start_offset -= bytes_actually_read;
    }

    // Read data out
    bytes_to_read = MIN(sm_context.log_read.length, SYSHAL_USB_PACKET_SIZE);

    ret = fs_read(sm_context.log_read.file_handle, read_buffer, bytes_to_read, &bytes_actually_read);
    DEBUG_PR_TRACE("Reading from Log File");
    printf("Contents: ");
    for (uint32_t i = 0; i < bytes_actually_read; ++i)
        printf("%02X ", read_buffer[i]);
    printf("\r\n");

    if (FS_NO_ERROR != ret)
    {
        Throw(EXCEPTION_FS_ERROR);
    }

    sm_context.log_read.length -= bytes_actually_read;

    buffer_write_advance(&config_if_send_buffer, bytes_actually_read);
    config_if_send_priv(&config_if_send_buffer);

    if (sm_context.log_read.length) // Is there still data to send?
    {
        config_if_timeout_reset(); // Reset the message timeout counter
    }
    else
    {
        // We have sent all the data
        fs_close(sm_context.log_read.file_handle); // Close the file
        message_set_state(SM_MESSAGE_STATE_IDLE);
    }

}

////////////////////////////////////////////////////////////////////////////////
////////////////////////// MESSAGE STATE EXECUTION CODE ////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void config_if_session_cleanup(void)
{
    buffer_reset(&config_if_send_buffer);
    buffer_reset(&config_if_receive_buffer);
    config_if_tx_pending = false;
    config_if_rx_queued = false;
}

int config_if_event_handler(config_if_event_t * event)
{
    // This is called from an interrupt so we'll keep it short
    switch (event->id)
    {

        case CONFIG_IF_EVENT_SEND_COMPLETE:
            buffer_read_advance(&config_if_send_buffer, event->send.size); // Remove it from the buffer
            config_if_tx_pending = false;
            //DEBUG_PR_TRACE("Send, Size: %lu", event->send.size);
            break;

        case CONFIG_IF_EVENT_RECEIVE_COMPLETE:
            buffer_write_advance(&config_if_receive_buffer, event->receive.size); // Store it in the buffer
            config_if_rx_queued = false;
            //DEBUG_PR_TRACE("Receive, Size: %lu", event->receive.size);
            break;

        case CONFIG_IF_EVENT_CONNECTED:
            DEBUG_PR_TRACE("CONFIG_IF_EVENT_CONNECTED");
            config_if_session_cleanup(); // Clean up any previous session
            config_if_timeout_reset(); // Reset our timeout counter
            config_if_connected = true;
            break;

        case CONFIG_IF_EVENT_DISCONNECTED:
            DEBUG_PR_TRACE("CONFIG_IF_EVENT_DISCONNECTED");
            // Clear all pending transmissions/receptions
            config_if_session_cleanup();
            config_if_connected = false;
            syshal_gps_bridging = false;
            break;

    }

    return CONFIG_IF_NO_ERROR;
}

static void message_idle_state(void)
{
    cmd_t * req;
    uint32_t length = buffer_read(&config_if_receive_buffer, (uintptr_t *)&req);
    if (length) // Is a message waiting to be processed
    {

        // Then process any pending event here, outside of an interrupt

        // Mark this message as received. This maybe a bit preemptive but it
        // is assumed the request handlers will handle the message appropriately
        buffer_read_advance(&config_if_receive_buffer, length);

        switch (req->h.cmd)
        {
            case CMD_CFG_READ_REQ:
                DEBUG_PR_INFO("CFG_READ_REQ");
                cfg_read_req(req, length);
                break;

            case CMD_CFG_WRITE_REQ:
                DEBUG_PR_INFO("CFG_WRITE_REQ");
                cfg_write_req(req, length);
                break;

            case CMD_CFG_SAVE_REQ:
                DEBUG_PR_INFO("CFG_SAVE_REQ");
                cfg_save_req(req, length);
                break;

            case CMD_CFG_RESTORE_REQ:
                DEBUG_PR_INFO("CFG_RESTORE_REQ");
                cfg_restore_req(req, length);
                break;

            case CMD_CFG_ERASE_REQ:
                DEBUG_PR_INFO("CFG_ERASE_REQ");
                cfg_erase_req(req, length);
                break;

            case CMD_CFG_PROTECT_REQ:
                DEBUG_PR_INFO("CFG_PROTECT_REQ");
                cfg_protect_req(req, length);
                break;

            case CMD_CFG_UNPROTECT_REQ:
                DEBUG_PR_INFO("CFG_UNPROTECT_REQ");
                cfg_unprotect_req(req, length);
                break;

            case CMD_GPS_WRITE_REQ:
                DEBUG_PR_INFO("GPS_WRITE_REQ");
                gps_write_req(req, length);
                break;

            case CMD_GPS_READ_REQ:
                DEBUG_PR_INFO("GPS_READ_REQ");
                gps_read_req(req, length);
                break;

            case CMD_GPS_CONFIG_REQ:
                DEBUG_PR_INFO("GPS_CONFIG_REQ");
                gps_config_req(req, length);
                break;

            case CMD_BLE_CONFIG_REQ:
                DEBUG_PR_INFO("BLE_CONFIG_REQ");
                ble_config_req(req, length);
                break;

            case CMD_BLE_WRITE_REQ:
                DEBUG_PR_INFO("BLE_WRITE_REQ");
                ble_write_req(req, length);
                break;

            case CMD_BLE_READ_REQ:
                DEBUG_PR_INFO("BLE_READ_REQ");
                ble_read_req(req, length);
                break;

            case CMD_STATUS_REQ:
                DEBUG_PR_INFO("STATUS_REQ");
                status_req(req, length);
                break;

            case CMD_FW_SEND_IMAGE_REQ:
                DEBUG_PR_INFO("FW_SEND_IMAGE_REQ");
                fw_send_image_req(req, length);
                break;

            case CMD_FW_APPLY_IMAGE_REQ:
                DEBUG_PR_INFO("FW_APPLY_IMAGE_REQ");
                fw_apply_image_req(req, length);
                break;

            case CMD_RESET_REQ:
                DEBUG_PR_INFO("RESET_REQ");
                reset_req(req, length);
                break;

            case CMD_BATTERY_STATUS_REQ:
                DEBUG_PR_INFO("BATTERY_STATUS_REQ");
                battery_status_req(req, length);
                break;

            case CMD_LOG_CREATE_REQ:
                DEBUG_PR_INFO("LOG_CREATE_REQ");
                log_create_req(req, length);
                break;

            case CMD_LOG_ERASE_REQ:
                DEBUG_PR_INFO("LOG_ERASE_REQ");
                log_erase_req(req, length);
                break;

            case CMD_LOG_READ_REQ:
                DEBUG_PR_INFO("LOG_READ_REQ");
                log_read_req(req, length);
                break;

            default:
                DEBUG_PR_WARN("Unhandled command: id %d", req->h.cmd);
                // Don't return an error. Fail silent
                break;
        }

    }
    else
    {
        config_if_receive_priv();
    }
}

void state_message_exception_handler(CEXCEPTION_T e)
{
    switch (e)
    {
        case EXCEPTION_BAD_SYS_CONFIG_ERROR_CONDITION:
            DEBUG_PR_ERROR("EXCEPTION_BAD_SYS_CONFIG_ERROR_CONDITION");
            break;

        case EXCEPTION_REQ_WRONG_SIZE:
            DEBUG_PR_ERROR("EXCEPTION_REQ_WRONG_SIZE");
            break;

        case EXCEPTION_TX_BUFFER_FULL:
            DEBUG_PR_ERROR("EXCEPTION_TX_BUFFER_FULL");
            break;

        case EXCEPTION_TX_BUSY:
            DEBUG_PR_ERROR("EXCEPTION_TX_BUSY");
            break;

        case EXCEPTION_RX_BUFFER_EMPTY:
            DEBUG_PR_ERROR("EXCEPTION_RX_BUFFER_EMPTY");
            break;

        case EXCEPTION_RX_BUFFER_FULL:
            DEBUG_PR_ERROR("EXCEPTION_RX_BUFFER_FULL");
            break;

        case EXCEPTION_PACKET_WRONG_SIZE:
            DEBUG_PR_ERROR("EXCEPTION_PACKET_WRONG_SIZE");
            break;

        case EXCEPTION_GPS_SEND_ERROR:
            DEBUG_PR_ERROR("EXCEPTION_GPS_SEND_ERROR");
            break;

        case EXCEPTION_FS_ERROR:
            DEBUG_PR_ERROR("EXCEPTION_FS_ERROR");
            break;

        default:
            DEBUG_PR_ERROR("Unknown message exception");
            break;
    }
}

static inline void config_if_timeout_reset(void)
{
    config_if_message_timeout = syshal_time_get_ticks_ms();
}

static void message_set_state(sm_message_state_t s)
{
    config_if_timeout_reset();
    message_state = s;
}

static void handle_config_if_messages(void)
{
    CEXCEPTION_T e = CEXCEPTION_NONE;

    // Has a message timeout occured?
    if ((syshal_time_get_ticks_ms() - config_if_message_timeout) > SM_MESSAGE_INACTIVITY_TIMEOUT_MS)
    {
        DEBUG_PR_WARN("State: %d, MESSAGE TIMEOUT", message_state);
        message_set_state(SM_MESSAGE_STATE_IDLE); // Return to the idle state
        config_if_session_cleanup(); // Clear any pending messages
    }

    // Don't allow the processing of anymore messages until we have a free transmit buffer
    if (config_if_tx_pending)
        return;

    Try
    {

        switch (message_state)
        {
            case SM_MESSAGE_STATE_IDLE: // No message is currently being handled
                message_idle_state();
                config_if_timeout_reset(); // Reset the timeout counter
                break;

            case SM_MESSAGE_STATE_CFG_READ_NEXT:
                cfg_read_next_state();
                break;

            case SM_MESSAGE_STATE_CFG_WRITE_NEXT:
                cfg_write_next_state();
                break;

            case SM_MESSAGE_STATE_CFG_WRITE_ERROR:
                cfg_write_error_state();
                break;

            case SM_MESSAGE_STATE_GPS_WRITE_NEXT:
                gps_write_next_state();
                break;

            case SM_MESSAGE_STATE_GPS_READ_NEXT:
                gps_read_next_state();
                break;

            case SM_MESSAGE_STATE_BLE_READ_NEXT:
                ble_read_next_state();
                break;

            case SM_MESSAGE_STATE_BLE_WRITE_NEXT:
                ble_write_next_state();
                break;

            case SM_MESSAGE_STATE_LOG_READ_NEXT:
                log_read_next_state();
                break;

            case SM_MESSAGE_STATE_FW_SEND_IMAGE_NEXT:
                fw_send_image_next_state();
                break;

            default:
                // TODO: add an illegal error state here for catching
                // invalid state changes
                break;
        }

    } Catch (e)
    {
        state_message_exception_handler(e);
    }
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////////// STATE EXECUTION CODE /////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void boot_state(void)
{
    setup_buffers();

    // Initialize all configured peripherals
    syshal_rtc_init();

    syshal_gpio_init(GPIO_LED1_GREEN);
    syshal_gpio_init(GPIO_LED2_RED);

    syshal_uart_init(UART_1);
    syshal_uart_init(UART_2);
//    syshal_uart_init(UART_3);

    syshal_spi_init(SPI_1);
    syshal_spi_init(SPI_2);

    syshal_i2c_init(I2C_1);
    syshal_i2c_init(I2C_2);

    syshal_flash_init(0, SPI_FLASH);

#ifndef DUMMY_BATTERY_MONITOR
    syshal_batt_init();
#endif

    // Re/Set global vars
    syshal_gps_bridging = false;
    syshal_ble_bridging = false;

    // Print General System Info
    DEBUG_PR_SYS("Arribada Tracker Device");
    DEBUG_PR_SYS("Version:  %s", GIT_VERSION);
    DEBUG_PR_SYS("Compiled: %s %s With %s", COMPILE_DATE, COMPILE_TIME, COMPILER_NAME);

    config_if_init(CONFIG_IF_BACKEND_USB);

    // Load the file system
    fs_init(FS_DEVICE);
    fs_mount(FS_DEVICE, &file_system);

    int ret = fs_get_configuration_data();

    // Init the peripheral devices after configuration data has been collected
    //syshal_axl_init();
    syshal_gps_init();
    syshal_switch_init();

//    syshal_gps_shutdown();
//    while(1)
//    {
//        syshal_pmu_set_level(POWER_STOP);
//        syshal_uart_init(UART_2);
//        syshal_time_delay_ms(1000);
//        DEBUG_PR_TRACE("Device woken");
//        syshal_time_delay_ms(1000);
//    }

    if (!(FS_NO_ERROR == ret || FS_ERROR_FILE_NOT_FOUND == ret))
        Throw(EXCEPTION_FS_ERROR);

#ifndef DUMMY_BATTERY_MONITOR
    // If the battery is charging then the system shall transition to the BATTERY_CHARGING state
    if (syshal_batt_charging())
    {
        sm_set_state(SM_STATE_STANDBY_BATTERY_CHARGING);
        return;
    }

    // If the battery level is too low then the system shall transition to the BATTERY_LEVEL_LOW state
    int level = syshal_batt_level();
    if ( (level <= SYSHAL_BATT_LEVEL_LOW) && (level >= 0) )
    {
        sm_set_state(SM_STATE_STANDBY_BATTERY_LEVEL_LOW);
        return;
    }
#endif

    // If either the USB 5V input signal or the BLE reed switch are active and the battery level is sufficient then it shall be possible to transition directly to the PROVISIONING state.
    if (config_if_connected)
    {
        sm_set_state(SM_STATE_PROVISIONING);
        return;
    }

    if (!check_configuration_tags_set()) // Check that all configuration tags are set
        sm_set_state(SM_STATE_STANDBY_PROVISIONING_NEEDED); // Not all required configuration data is set
    else
    {
        // And a log file exists
        fs_handle_t file_system_handle;
        int ret = fs_open(file_system, &file_system_handle, FS_FILE_ID_LOG, FS_MODE_READONLY, NULL);

        if (FS_NO_ERROR == ret)
        {
            fs_close(file_system_handle);
            sm_set_state(SM_STATE_OPERATIONAL); // All systems go for standard operation
        }
        else
        {
            syshal_gps_shutdown();
            sm_set_state(SM_STATE_STANDBY_PROVISIONING_NEEDED); // We still need provisioning
        }
    }

}

void standby_battery_charging_state()
{
#ifndef DUMMY_BATTERY_MONITOR
    if (!syshal_batt_charging())
    {
        int level = syshal_batt_level();
        if ( (level <= SYSHAL_BATT_LEVEL_LOW) && (level >= 0) )
        {
            if (config_if_connected)
                sm_set_state(SM_STATE_PROVISIONING);
            else
                sm_set_state(SM_STATE_STANDBY_PROVISIONING_NEEDED);
        }
        else
        {
            sm_set_state(SM_STATE_STANDBY_BATTERY_LEVEL_LOW);
        }
    }
#endif
}

void standby_battery_level_low_state()
{
#ifndef DUMMY_BATTERY_MONITOR
    // If the battery is charging then the system shall transition to the BATTERY_CHARGING state
    if (syshal_batt_charging())
    {
        sm_set_state(SM_STATE_STANDBY_BATTERY_CHARGING);
        return;
    }
#endif
}

void standby_log_file_full_state()
{
    // Check to see if any state changes are required

    if (config_if_connected)
    {
        sm_set_state(SM_STATE_PROVISIONING); // Configuration interface connected
        return;
    }

#ifndef DUMMY_BATTERY_MONITOR
    // If the battery is charging then the system shall transition to the BATTERY_CHARGING state
    if (syshal_batt_charging())
    {
        sm_set_state(SM_STATE_STANDBY_BATTERY_CHARGING);
        return;
    }

    // If the battery level is too low then the system shall transition to the BATTERY_LEVEL_LOW state
    int level = syshal_batt_level();
    if ( (level <= SYSHAL_BATT_LEVEL_LOW) && (level >= 0) )
    {
        sm_set_state(SM_STATE_STANDBY_BATTERY_LEVEL_LOW);
        return;
    }
#endif
}

void standby_provisioning_needed_state()
{
// This sub-state is entered if the currently active configuration in RAM (upon being read from flash) is invalid, incomplete or has no sensors enabled.
// The system shall continue to monitor the battery level and charging status and may enter into the BATTERY_LEVEL_LOW or BATTERY_CHARGING sub-states.
// The system shall also monitor the USB 5V input signal and the BLE reed switch and if the battery level is sufficient it shall be possible to transition to the PROVISIONING state.

#ifndef GTEST
    // Blink an LED to indicate this state
    static uint32_t blinkTimer = 0;
    const uint32_t blinkTimeMs = 1000;

    if (syshal_time_get_ticks_ms() >= (blinkTimeMs + blinkTimer))
    {
        syshal_gpio_set_output_high(GPIO_LED2_RED);
        syshal_time_delay_ms(50);
        syshal_gpio_set_output_low(GPIO_LED2_RED);
        blinkTimer = syshal_time_get_ticks_ms();
    }
#endif

    if (config_if_connected)
    {
        syshal_gps_wake_up();
        sm_set_state(SM_STATE_PROVISIONING); // Not all tags are set, we need provisioning
        return;
    }

#ifndef DUMMY_BATTERY_MONITOR
    // If the battery is charging then the system shall transition to the BATTERY_CHARGING state
    if (syshal_batt_charging())
    {
        sm_set_state(SM_STATE_STANDBY_BATTERY_CHARGING);
        return;
    }

    // If the battery level is too low then the system shall transition to the BATTERY_LEVEL_LOW state
    int level = syshal_batt_level();
    if ( (level <= SYSHAL_BATT_LEVEL_LOW) && (level >= 0) )
    {
        sm_set_state(SM_STATE_STANDBY_BATTERY_LEVEL_LOW);
        return;
    }
#endif

    config_if_tick();
}

void standby_trigger_pending_state()
{

}

void provisioning_state(void)
{
#ifndef GTEST
    // Blink an LED to indicate this state
    static uint32_t blinkTimer = 0;
    const uint32_t blinkTimeMs = 200;

    if (syshal_time_get_ticks_ms() >= (blinkTimeMs + blinkTimer))
    {
        syshal_gpio_set_output_high(GPIO_LED2_RED);
        syshal_time_delay_ms(50);
        syshal_gpio_set_output_low(GPIO_LED2_RED);
        blinkTimer = syshal_time_get_ticks_ms();
    }
#endif

    config_if_tick();

    handle_config_if_messages(); // Process any config_if messages

    // Has our configuration interface been disconnected?
    if (!config_if_connected)
    {
        // If so we should change state
        if (check_configuration_tags_set())
        {
            // And a log file exists
            fs_handle_t file_system_handle;
            int ret = fs_open(file_system, &file_system_handle, FS_FILE_ID_LOG, FS_MODE_READONLY, NULL);

            if (FS_NO_ERROR == ret)
            {
                fs_close(file_system_handle);
                sm_set_state(SM_STATE_OPERATIONAL); // All systems go for standard operation
            }
            else
            {
                syshal_gps_shutdown();
                sm_set_state(SM_STATE_STANDBY_PROVISIONING_NEEDED); // We still need provisioning
            }
        }
        else
        {
            syshal_gps_shutdown();
            sm_set_state(SM_STATE_STANDBY_PROVISIONING_NEEDED); // We still need provisioning
        }
    }
}

void operational_state(void)
{
    // If log file is not open
    if (NULL == log_file_handle)
    {
        int ret = fs_open(file_system, &log_file_handle, FS_FILE_ID_LOG, FS_MODE_WRITEONLY, NULL);

        switch (ret)
        {
            case FS_NO_ERROR:
                // All systems go
                // Flush any data that we previously may have in the buffers and start fresh
                buffer_reset(&logging_buffer);

                if (sys_config.sys_config_gps_log_position_enable.contents.enable)
                {
                    syshal_gps_wake_up();
                    // Clear the GPS buffer
                    uint8_t flush;
                    while (syshal_gps_receive_raw(&flush, 1))
                    {}
                    syshal_gps_shutdown();
                }
                else
                {
                    syshal_gps_shutdown();
                }
                break;

            case FS_ERROR_FILE_NOT_FOUND:
                log_file_handle = NULL;
                sm_set_state(SM_STATE_STANDBY_PROVISIONING_NEEDED); // We still need provisioning
                break;

            case FS_ERROR_FILE_PROTECTED: // We never lock the log file so this shouldn't occur
            default:
                log_file_handle = NULL;
                Throw(EXCEPTION_FS_ERROR);
                break;
        }
    }

    // If GPS bridging is disabled and GPS logging enabled
    if ( (!syshal_gps_bridging) && (sys_config.sys_config_gps_log_position_enable.contents.enable) )
        syshal_gps_tick(); // Process GPS messages

    // Is there any data waiting to be written to the log file?
    uint8_t * read_buffer;
    uint32_t length = buffer_read(&logging_buffer, (uintptr_t *)&read_buffer);

    if (length)
    {
        uint32_t bytes_written;
        int ret = fs_write(log_file_handle, read_buffer, length, &bytes_written);
        DEBUG_PR_TRACE("Writing to Log File");
        printf("Contents: ");
        for (uint32_t i = 0; i < length; ++i)
            printf("%02X ", read_buffer[i]);
        printf("\r\n");

        switch (ret)
        {
            case FS_NO_ERROR:
                buffer_read_advance(&logging_buffer, length);
                break;

            case FS_ERROR_FILESYSTEM_FULL: // Our log file is full
                fs_close(log_file_handle);
                sm_set_state(SM_STATE_STANDBY_LOG_FILE_FULL);
                break;

            case FS_ERROR_FLASH_MEDIA:
            default:
                fs_close(log_file_handle);
                Throw(EXCEPTION_FS_ERROR);
                break;
        }
    }

    config_if_tick();

    // Check to see if any state changes are required
    if (config_if_connected)
    {
        if (log_file_handle)
        {
            fs_close(log_file_handle);
            log_file_handle = NULL;
        }
        sm_set_state(SM_STATE_PROVISIONING); // Configuration interface connected
        return;
    }

#ifndef DUMMY_BATTERY_MONITOR
    // If the battery is charging then the system shall transition to the BATTERY_CHARGING state
    if (syshal_batt_charging())
    {
        if (log_file_handle)
        {
            fs_close(log_file_handle);
            log_file_handle = NULL;
        }
        sm_set_state(SM_STATE_STANDBY_BATTERY_CHARGING);
        return;
    }

    // If the battery level is too low then the system shall transition to the BATTERY_LEVEL_LOW state
    int level = syshal_batt_level();
    if ( (level <= SYSHAL_BATT_LEVEL_LOW) && (level >= 0) )
    {
        if (log_file_handle)
        {
            fs_close(log_file_handle);
            log_file_handle = NULL;
        }
        sm_set_state(SM_STATE_STANDBY_BATTERY_LEVEL_LOW);
        return;
    }
#endif

    syshal_pmu_set_level(POWER_SLEEP);

}
////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// STATE HANDLERS ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void state_exception_handler(CEXCEPTION_T e)
{
    switch (e)
    {
        case EXCEPTION_REQ_WRONG_SIZE:
            DEBUG_PR_ERROR("EXCEPTION_REQ_WRONG_SIZE");
            break;

        case EXCEPTION_RESP_TX_PENDING:
            DEBUG_PR_ERROR("EXCEPTION_RESP_TX_PENDING");
            break;

        case EXCEPTION_TX_BUFFER_FULL:
            DEBUG_PR_ERROR("EXCEPTION_TX_BUFFER_FULL");
            break;

        case EXCEPTION_TX_BUSY:
            DEBUG_PR_ERROR("EXCEPTION_TX_BUSY");
            break;

        case EXCEPTION_RX_BUFFER_EMPTY:
            DEBUG_PR_ERROR("EXCEPTION_RX_BUFFER_EMPTY");
            break;

        case EXCEPTION_RX_BUFFER_FULL:
            DEBUG_PR_ERROR("EXCEPTION_RX_BUFFER_FULL");
            break;

        case EXCEPTION_BAD_SYS_CONFIG_ERROR_CONDITION:
            DEBUG_PR_ERROR("EXCEPTION_BAD_SYS_CONFIG_ERROR_CONDITION");
            break;

        case EXCEPTION_PACKET_WRONG_SIZE:
            DEBUG_PR_ERROR("EXCEPTION_PACKET_WRONG_SIZE");
            break;

        case EXCEPTION_GPS_SEND_ERROR:
            DEBUG_PR_ERROR("EXCEPTION_GPS_SEND_ERROR");
            break;

        case EXCEPTION_FS_ERROR:
            DEBUG_PR_ERROR("EXCEPTION_FS_ERROR");
            break;

        case EXCEPTION_SPI_ERROR:
            DEBUG_PR_ERROR("EXCEPTION_SPI_ERROR");
            break;


        default:
            DEBUG_PR_ERROR("Unknown state exception %d", e);
            break;
    }
}

sm_state_t sm_get_state(void)
{
    return state;
}

void sm_set_state(sm_state_t s)
{
    static sm_state_t pastState = SM_STATE_BOOT;
    state = s;
    if (pastState != s)
        DEBUG_PR_INFO("Switching state to: %s from: %s", sm_state_str[s], sm_state_str[pastState]);
    pastState = s;
}

void sm_iterate(void)
{
    CEXCEPTION_T e = CEXCEPTION_NONE;

    Try
    {
        switch (state)
        {
            case SM_STATE_BOOT:
                boot_state();
                break;

            case SM_STATE_STANDBY_BATTERY_CHARGING:
                standby_battery_charging_state();
                break;
            case SM_STATE_STANDBY_BATTERY_LEVEL_LOW:
                standby_battery_level_low_state();
                break;
            case SM_STATE_STANDBY_LOG_FILE_FULL:
                standby_log_file_full_state();
                break;
            case SM_STATE_STANDBY_PROVISIONING_NEEDED:
                standby_provisioning_needed_state();
                break;
            case SM_STATE_STANDBY_TRIGGER_PENDING:
                standby_trigger_pending_state();
                break;

            case SM_STATE_PROVISIONING:
                provisioning_state();
                break;

            case SM_STATE_OPERATIONAL:
                operational_state();
                break;

            default:
                // TODO: add an illegal error state here for catching
                // invalid state changes
                break;
        }
    } Catch (e)
    {
        state_exception_handler(e);
    }
}
