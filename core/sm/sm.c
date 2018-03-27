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
#include "debug.h"
#include "exceptions.h"
#include "sm.h"
#include "sys_config.h"
#include "syshal_batt.h"
#include "syshal_gpio.h"
#include "syshal_gps.h"
#include "syshal_i2c.h"
#include "syshal_spi.h"
#include "syshal_uart.h"
#include "syshal_usb.h"
#include "version.h"

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// MAIN STATES ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static sm_state_t state = SM_STATE_BOOT; // Default starting state

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
} sm_message_state_t;
static sm_message_state_t message_state = SM_MESSAGE_STATE_IDLE;

// State specific context
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
            uint32_t  length;
        } gps_write;
    };
} sm_context_t;
static sm_context_t sm_context;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// GLOBALS /////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static volatile bool     config_if_tx_pending;
static volatile bool     config_if_rx_queued;
static bool              syshal_gps_bridging;
static volatile buffer_t config_if_send_buffer;
static volatile buffer_t config_if_receive_buffer;
static volatile uint8_t  config_if_send_buffer_pool[SYSHAL_USB_PACKET_SIZE * 2];
static volatile uint8_t  config_if_receive_buffer_pool[SYSHAL_USB_PACKET_SIZE];
static uint32_t          config_if_message_timeout;

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
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// HELPER FUNCTIONS ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////

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
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// CFG_RESTORE /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void cfg_restore_req(cmd_t * req, uint16_t size)
{
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);
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

    if (CFG_READ_REQ_READ_ALL == req->p.cmd_cfg_erase_req.configuration_tag) // Erase all configuration tags
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
                resp->p.cmd_generic_resp.error_code = CMD_ERROR_UNKNOWN;
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
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// CFG_UNPROTECT /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void cfg_unprotect_req(cmd_t * req, uint16_t size)
{
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);
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
//    // Check request size is correct
//    if (CMD_SIZE(cmd_gps_read_req_t) != size)
//        Throw(EXCEPTION_REQ_WRONG_SIZE);
//
//    // Generate and send response
//    cmd_t * resp = (cmd_t *) tx_buffer_working;
//    CMD_SET_HDR(resp, CMD_GPS_READ_RESP);
//
//    resp->p.cmd_gps_read_resp.length = syshal_gps_receive_raw(resp->p.cmd_gps_read_resp.bytes, req->p.cmd_gps_read_req.length);
//    resp->p.cmd_gps_read_resp.error_code = CMD_NO_ERROR;
//
//    config_if_send_priv((uint8_t *) resp, CMD_SIZE(cmd_gps_read_resp_t));
}

static void gps_config_req(cmd_t * req, uint16_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_gps_config_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    syshal_gps_bridging = req->p.cmd_gps_config_req.enable; // Disable or enable GPS bridging

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
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);
}

static void ble_write_req(cmd_t * req, uint16_t size)
{
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);
}

static void ble_read_req(cmd_t * req, uint16_t size)
{
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);
}

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
    resp->p.cmd_status_resp.firmware_version = 0;
    resp->p.cmd_status_resp.firmware_checksum = 0;
    resp->p.cmd_status_resp.configuration_format_version = 0;

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_status_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

static void fw_send_image_req(cmd_t * req, uint16_t size)
{
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);
}

static void fw_apply_image_req(cmd_t * req, uint16_t size)
{
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);
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

#ifdef __CORTEX_M // FIXME: find and replace with the ARM macro
    NVIC_SystemReset();
#endif

    // If a system reset isn't available then block until a watchdog reset
    for (;;) {}
}

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

    DEBUG_PR_WARN("%s() NOT IMPLEMENTED, responding with spoof data", __FUNCTION__);

    resp->p.cmd_battery_status_resp.error_code = CMD_NO_ERROR;
    resp->p.cmd_battery_status_resp.charging_indicator = 1;
    resp->p.cmd_battery_status_resp.charge_level = 100;

    /*
    resp->p.cmd_battery_status_resp.error_code = CMD_NO_ERROR;
    resp->p.cmd_battery_status_resp.charging_indicator = syshal_batt_charging();
    resp->p.cmd_battery_status_resp.charge_level = syshal_batt_level();
    */

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_battery_status_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

static void log_create_req(cmd_t * req, uint16_t size)
{
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);
}

static void log_erase_req(cmd_t * req, uint16_t size)
{
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);
}

static void log_read_req(cmd_t * req, uint16_t size)
{
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);
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
            DEBUG_PR_TRACE("Send, Size: %lu", event->send.size);
            break;

        case CONFIG_IF_EVENT_RECEIVE_COMPLETE:
            buffer_write_advance(&config_if_receive_buffer, event->receive.size); // Store it in the buffer
            config_if_rx_queued = false;
            DEBUG_PR_TRACE("Receive, Size: %lu", event->receive.size);
            break;

        case CONFIG_IF_EVENT_CONNECTED:
            DEBUG_PR_TRACE("CONFIG_IF_EVENT_CONNECTED");
            sm_set_state(SM_STATE_PROVISIONING);
            break;

        case CONFIG_IF_EVENT_DISCONNECTED:
            DEBUG_PR_TRACE("CONFIG_IF_EVENT_DISCONNECTED");
            // Clear all pending transmissions/receptions
            config_if_session_cleanup();
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

    // If this is our first time entering the function setup the timeout counter
    if (0 == config_if_message_timeout)
        config_if_timeout_reset();

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
    syshal_gpio_init(GPIO_LED3);
    syshal_gpio_init(GPIO_LED4);
    syshal_gpio_init(GPIO_LED5);
    syshal_gpio_init(GPIO_LED6);
    syshal_uart_init(UART_1);
    //syshal_uart_init(UART_3);
    //syshal_uart_init(UART_4);
    syshal_spi_init(SPI_1);
    syshal_spi_init(SPI_2);
    syshal_i2c_init(I2C_1);
    syshal_i2c_init(I2C_2);
    //syshal_batt_init(I2C_1);
    //syshal_gps_init();
    //syshal_usb_init();

    config_if_init(CONFIG_IF_BACKEND_USB);

    // Print General System Info
    DEBUG_PR_SYS("Arribada Tracker Device");
    DEBUG_PR_SYS("Version:  %s", GIT_VERSION);
    DEBUG_PR_SYS("Compiled: %s %s With %s", COMPILE_DATE, COMPILE_TIME, COMPILER_NAME);

    syshal_gpio_set_output_high(GPIO_LED3); // Indicate boot passed

    // If the battery is charging then the system shall transition to the BATTERY_CHARGING state
    //if (syshal_batt_charging())
    //    sm_set_state(SM_STATE_STANDBY_BATTERY_CHARGING);

    // If the battery level is too low then the system shall transition to the BATTERY_LEVEL_LOW state
    //if (syshal_batt_state() == POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL)
    //    sm_set_state(SM_STATE_STANDBY_BATTERY_LEVEL_LOW);

    // If either the USB 5V input signal or the BLE reed switch are active and the battery level is sufficient then it shall be possible to transition directly to the PROVISIONING state.
    //if (syshal_gpio_get_input(GPIO_USB_DETECT) || syshal_gpio_get_input(GPIO_BLE_REED))
    //    sm_set_state(SM_STATE_PROVISIONING);

    // Generate spoof time/date data
    sys_config_rtc_current_date_and_time_t date_time;
    date_time.contents.day = 1;
    date_time.contents.month = 2;
    date_time.contents.year = 3;
    date_time.contents.hours = 4;
    date_time.contents.minutes = 5;
    date_time.contents.seconds = 6;
    sys_config_set(SYS_CONFIG_TAG_RTC_CURRENT_DATE_AND_TIME, &date_time.contents, SYS_CONFIG_TAG_DATA_SIZE(sys_config_rtc_current_date_and_time_t));

    // Otherwise, if no configuration file is present or the current configuration is invalid then the system shall transition to the PROVISIONING_NEEDED sub-state.

    // Check that all configuration tags are set
    bool tag_not_set = false;
    uint16_t tag = 0;
    uint16_t last_index = 0;

    while (!sys_config_iterate(&tag, &last_index))
    {
        int return_code = sys_config_get(tag, NULL);

        if (SYS_CONFIG_ERROR_TAG_NOT_SET == return_code)
        {
            DEBUG_PR_WARN("Tag 0x%04X not set", tag);
            tag_not_set = true;
        }
    }

    if (tag_not_set)
        sm_set_state(SM_STATE_STANDBY_PROVISIONING_NEEDED); // Not all tags are set, we need provisioning
    else
        sm_set_state(SM_STATE_OPERATIONAL); // All systems go for standard operation

}

void standby_battery_charging_state()
{

}

void standby_battery_level_low_state()
{

}

void standby_log_file_full_state()
{

}

void standby_provisioning_needed_state()
{
// This sub-state is entered if the currently active configuration in RAM (upon being read from flash) is invalid, incomplete or has no sensors enabled.
// The system shall continue to monitor the battery level and charging status and may enter into the BATTERY_LEVEL_LOW or BATTERY_CHARGING sub-states.
// The system shall also monitor the USB 5V input signal and the BLE reed switch and if the battery level is sufficient it shall be possible to transition to the PROVISIONING state.

    // Blink an LED to indicate this state
    static uint32_t blinkTimer = 0;
    const uint32_t blinkTimeMs = 1000;

    if (syshal_time_get_ticks_ms() >= (blinkTimeMs + blinkTimer))
    {
        syshal_gpio_set_output_high(GPIO_LED4);
        syshal_time_delay_ms(50);
        syshal_gpio_set_output_low(GPIO_LED4);
        blinkTimer = syshal_time_get_ticks_ms();
    }

    sm_set_state(SM_STATE_PROVISIONING); // FIXME: Temporary state change as USB detection is not implemented

    // If the battery is charging then the system shall transition to the BATTERY_CHARGING state
    //if (syshal_batt_charging())
    //    sm_set_state(SM_STATE_STANDBY_BATTERY_CHARGING);

    // If the battery level is too low then the system shall transition to the BATTERY_LEVEL_LOW state
    //if (syshal_batt_state() == POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL)
    //    sm_set_state(SM_STATE_STANDBY_BATTERY_LEVEL_LOW);
}

void standby_trigger_pending_state()
{

}

void provisioning_state(void)
{
    syshal_gpio_set_output_high(GPIO_LED4); // Indicate what state we're in

    handle_config_if_messages();
}

void operational_state(void)
{

}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// STATE HANDLERS ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void state_exception_handler(CEXCEPTION_T e)
{
    switch (e)
    {
        default:
            DEBUG_PR_ERROR("Unknown state exception");
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