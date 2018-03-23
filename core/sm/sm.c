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
/////////////////////////////// Local variables ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

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

static sm_state_t state = SM_STATE_BOOT; // Default starting state

static uint8_t rx_buffer[SYSHAL_USB_PACKET_SIZE]; // RX buffer used by config_if

static volatile bool config_if_tx_pending = false; // Is a transmit pending
static volatile bool config_if_rx_pending = false; // Is a receive pending
static volatile uint16_t config_if_rx_size; // What was the size of the last receive in bytes
static bool syshal_gps_bridging = false; // Is a transmit pending

// Buffers
static buffer_t config_if_send_buffer;
//static buffer_t config_if_receive_buffer;

static uint8_t config_if_send_buffer_pool[SYSHAL_USB_PACKET_SIZE * 2];
//static uint8_t config_if_receive_buffer_pool[SYSHAL_USB_PACKET_SIZE];

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// PROTOTYPES //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// STARTUP ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void setup_buffers(void)
{
    buffer_init_policy(pool, &config_if_send_buffer, (uintptr_t) &config_if_send_buffer_pool[0], sizeof(config_if_send_buffer_pool), 2);
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// HELPER FUNCTIONS ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void config_if_send_priv(buffer_t * buffer)
{
    while (config_if_tx_pending) // FIXME: implement timeout
    {} // Wait for previous transmission to be sent

    uintptr_t addr;
    uint32_t length = buffer_read(buffer, &addr);

    if (length)
    {
        DEBUG_PR_TRACE("Sending packet of size %lu", length);

        config_if_tx_pending = true;
        config_if_send((uint8_t *) addr, length); // Send response
    }
}

static void config_if_receive_blocking(uint8_t * data, uint32_t size)
{
    config_if_receive(data, size);

    while (!config_if_rx_pending) // FIXME: implement timeout
    {} // Wait for data to be received

    config_if_rx_pending = false; // Mark this packet as received
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// REQUEST HANDLERS //////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void cfg_read_req(cmd_t * req, uint16_t size)
{

    // Check request size is correct
    if (CMD_SIZE(cmd_cfg_read_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_CFG_READ_RESP);

    if (CFG_READ_REQ_READ_ALL == req->p.cmd_cfg_read_req.configuration_tag) // Read all configuration tags
    {
        DEBUG_PR_WARN("READ ALL TAGS IN %s() NOT IMPLEMENTED", __FUNCTION__);
        return;
    }
    else // Read just one tag
    {
        uint8_t tag_value[SYS_CONFIG_MAX_CONFIGURATION_SIZE];
        uint16_t tag = req->p.cmd_cfg_read_req.configuration_tag;

        int tag_length = sys_config_get(tag, &tag_value[0]);

        resp->p.cmd_cfg_read_resp.length = 0;

        if (SYS_CONFIG_ERROR_INVALID_TAG == tag_length)  // Tag is not valid. Return an error code
        {
            resp->p.cmd_cfg_read_resp.error_code = CMD_ERROR_INVALID_CONFIG_TAG;
        }
        else if (SYS_CONFIG_ERROR_TAG_NOT_SET == tag_length)  // Tag is not set. Return an error code
        {
            resp->p.cmd_cfg_read_resp.error_code = CMD_ERROR_CONFIG_TAG_NOT_SET;
        }
        else if (tag_length < 0) // Unknown/Unhandled error
        {
            resp->p.cmd_cfg_read_resp.error_code = CMD_ERROR_UNKNOWN;
        }
        else // Tag is valid
        {
            resp->p.cmd_cfg_read_resp.error_code = CMD_NO_ERROR;
            resp->p.cmd_cfg_read_resp.length = sizeof(tag) + tag_length; // We will be sending a tag along with its value
        }

        buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_cfg_read_resp_t));
        config_if_send_priv(&config_if_send_buffer); // Send response

        if (tag_length >= 0) // If there was no error
        {

            uint8_t * data_buffer;
            if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&data_buffer))
                Throw(EXCEPTION_TX_BUFFER_FULL);

            // Send the configuration tag's value
            memcpy(&data_buffer[0], &tag, sizeof(tag)); // Start of packet is the configuration tag itself
            memcpy(&data_buffer[sizeof(tag)], &tag_value[0], tag_length);  // And now for the tag_value

            buffer_write_advance(&config_if_send_buffer, resp->p.cmd_cfg_read_resp.length);
            config_if_send_priv(&config_if_send_buffer); // Send tag_value
        }

    }

}

void cfg_write_req(cmd_t * req, uint16_t size)
{

    // Check request size is correct
    if (CMD_SIZE(cmd_cfg_write_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    uint32_t length = req->p.cmd_cfg_write_req.length;

    // Length is not zero
    if (!length)
        return;

    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    resp->p.cmd_cfg_read_resp.error_code = CMD_NO_ERROR;

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer); // Send response

    if (CFG_READ_REQ_READ_ALL == length) // Read all configuration tags
    {
        DEBUG_PR_WARN("WRITE ALL TAGS IN %s() NOT IMPLEMENTED", __FUNCTION__);
        return;
    }
    else // Write just one tag
    {
        config_if_receive_blocking(rx_buffer, length);

        uint16_t tag = ((uint16_t)rx_buffer[1] << 8) | rx_buffer[0];
        DEBUG_PR_TRACE("Setting tag 0x%04X", tag);
        sys_config_set(tag, &rx_buffer[sizeof(tag)], length - sizeof(tag));
    }

    // Return CFG_WRITE_CNF to mark completion of transfer
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_CFG_WRITE_CNF);

    resp->p.cmd_cfg_write_cnf.error_code = CMD_NO_ERROR;

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_cfg_write_cnf_t));
    config_if_send_priv(&config_if_send_buffer); // Send confirmation

}

void cfg_save_req(cmd_t * req, uint16_t size)
{
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);
}

void cfg_restore_req(cmd_t * req, uint16_t size)
{
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);
}

void cfg_erase_req(cmd_t * req, uint16_t size)
{
    /*
    // Check request size is correct
    if (CMD_SIZE(cmd_cfg_erase_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    if (CFG_READ_REQ_READ_ALL == req->p.cmd_cfg_erase_req.configuration_tag) // Erase all configuration tags
    {
        DEBUG_PR_WARN("ERASE ALL TAGS IN %s() NOT IMPLEMENTED", __FUNCTION__);
        return;
    }
    else
    {
        // Erase just one configuration tag
        cmd_t * resp = (cmd_t *) tx_buffer_working;
        CMD_SET_HDR(resp, CMD_GENERIC_RESP);

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

        config_if_send_priv((uint8_t *) resp, CMD_SIZE(cmd_generic_resp_t));
    }
    */
}

void cfg_protect_req(cmd_t * req, uint16_t size)
{
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);
}

void cfg_unprotect_req(cmd_t * req, uint16_t size)
{
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);
}

void gps_write_req(cmd_t * req, uint16_t size)
{
//    // Check request size is correct
//    if (CMD_SIZE(cmd_gps_write_req_t) != size)
//        Throw(EXCEPTION_REQ_WRONG_SIZE);
//
//
//    int return_code = syshal_gps_send_raw(req->p.cmd_gps_write_req.bytes, req->p.cmd_gps_write_req.length);
//
//    // Generate and send response
//    cmd_t * resp = (cmd_t *) &tx_buffer[0];
//    CMD_SET_HDR(resp, CMD_GENERIC_RESP);
//
//    switch (return_code)
//    {
//        case SYSHAL_GPS_NO_ERROR:
//            resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
//            break;
//
//        case SYSHAL_GPS_ERROR_TIMEOUT:
//            resp->p.cmd_generic_resp.error_code = CMD_ERROR_TIMEOUT;
//            break;
//
//        case SYSHAL_GPS_ERROR_BUSY:
//        case SYSHAL_GPS_ERROR_DEVICE:
//        default:
//            resp->p.cmd_generic_resp.error_code = CMD_ERROR_UNKNOWN;
//            break;
//    }
//
//    config_if_send_priv((uint8_t *) resp, CMD_SIZE(cmd_generic_resp_t));
}

void gps_read_req(cmd_t * req, uint16_t size)
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

void gps_config_req(cmd_t * req, uint16_t size)
{
    /*
    // Check request size is correct
    if (CMD_SIZE(cmd_gps_config_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    syshal_gps_bridging = req->p.cmd_gps_config_req.enable; // Disable or enable GPS bridging

    // Generate and send response
    cmd_t * resp = (cmd_t *) tx_buffer_working;
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;

    config_if_send_priv((uint8_t *) resp, CMD_SIZE(cmd_generic_resp_t));
    */
}

void ble_config_req(cmd_t * req, uint16_t size)
{
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);
}

void ble_write_req(cmd_t * req, uint16_t size)
{
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);
}

void ble_read_req(cmd_t * req, uint16_t size)
{
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);
}

void status_req(cmd_t * req, uint16_t size)
{
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);

    /*
    // Generate and send response
    cmd_t * resp;
    CMD_SET_HDR(resp, CMD_STATUS_RESP, sizeof(cmd_status_resp_t));

    resp->p.cmd_status_resp.error_code = CMD_NO_ERROR;
    resp->p.cmd_status_resp.firmware_version = ;
    resp->p.cmd_status_resp.firmware_checksum = ;
    resp->p.cmd_status_resp.configuration_format_version = ;
    */
}

void fw_send_image_req(cmd_t * req, uint16_t size)
{
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);
}

void fw_send_image_complete_cnf(cmd_t * req, uint16_t size)
{
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);
}

void fw_apply_image_req(cmd_t * req, uint16_t size)
{
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);
}

void reset_req(cmd_t * req, uint16_t size)
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

    //DEBUG_PR_INFO("A");

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer);

    //DEBUG_PR_INFO("B");

    // Wait for response to have been sent
    while (config_if_tx_pending)
    {}

    //DEBUG_PR_INFO("C");

#ifdef __CORTEX_M // FIXME: find and replace with the ARM macro
    NVIC_SystemReset();
#endif

    // If a system reset isn't available then block until a watchdog reset
    for (;;) {}
}

void battery_status_req(cmd_t * req, uint16_t size)
{
//    // Check request size is correct
//    if (size != CMD_SIZE_HDR)
//        Throw(EXCEPTION_REQ_WRONG_SIZE);
//
//    // If we're already currently responding to another request
//    if (config_if_tx_pending)
//        Throw(EXCEPTION_RESP_TX_PENDING);
//
//    DEBUG_PR_WARN("%s() NOT IMPLEMENTED, responding with spoof data", __FUNCTION__);
//
//    // Generate and send response
//    cmd_t * resp = (cmd_t *) &tx_buffer[0];
//    CMD_SET_HDR(resp, CMD_BATTERY_STATUS_RESP);
//
//    resp->p.cmd_battery_status_resp.error_code = CMD_NO_ERROR;
//    resp->p.cmd_battery_status_resp.charging_indicator = 1;
//    resp->p.cmd_battery_status_resp.charge_level = 100;
//
//    /*
//    resp->p.cmd_battery_status_resp.error_code = CMD_NO_ERROR;
//    resp->p.cmd_battery_status_resp.charging_indicator = syshal_batt_charging();
//    resp->p.cmd_battery_status_resp.charge_level = syshal_batt_level();
//    */
//
//    config_if_send_priv((uint8_t *) resp, CMD_SIZE(cmd_battery_status_resp_t));
}

void log_create_req(cmd_t * req, uint16_t size)
{
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);
}

void log_erase_req(cmd_t * req, uint16_t size)
{
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);
}

void log_read_req(cmd_t * req, uint16_t size)
{
    DEBUG_PR_WARN("%s NOT IMPLEMENTED", __FUNCTION__);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// CMD HANDLERS ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int config_if_event_handler(config_if_event_t * event)
{
    // This is called from an interrupt so we'll keep it short
    switch (event->id)
    {

        case CONFIG_IF_EVENT_SEND_COMPLETE:
            DEBUG_PR_TRACE("CONFIG_IF_EVENT_SEND_COMPLETE");
            buffer_read_advance(&config_if_send_buffer, event->send.size); // Remove it from the buffer
            config_if_tx_pending = false;
            break;

        case CONFIG_IF_EVENT_RECEIVE_COMPLETE:
            DEBUG_PR_TRACE("CONFIG_IF_EVENT_RECEIVE_COMPLETE, SIZE: %lu", event->receive.size);
            config_if_rx_pending = true; // Indicate that there is a packet in the RX buffer that needs processing
            config_if_rx_size = event->receive.size;// Store the size of the packet
            break;

        case CONFIG_IF_EVENT_CONNECTED:
            DEBUG_PR_TRACE("CONFIG_IF_EVENT_CONNECTED");
            break;

        case CONFIG_IF_EVENT_DISCONNECTED:
            DEBUG_PR_TRACE("CONFIG_IF_EVENT_DISCONNECTED");
            // Clear all pending transmissions/receptions
            buffer_reset(&config_if_send_buffer);
            config_if_tx_pending = false;
            config_if_rx_pending = false;
            config_if_rx_size = 0;
            syshal_gps_bridging = false;
            break;

    }

    return CONFIG_IF_NO_ERROR;
}

static void handle_config_if_requests(void)
{
    // Process any pending event outside of an interrupt
    if (config_if_rx_pending)
    {

        config_if_rx_pending = false; // Mark this message as received. This maybe a bit preemptive but it
        // is assumed the request handlers will receive the message appropriately

        cmd_t * req = (cmd_t *) &rx_buffer[0]; // Overlay command structure on receive buffer

        switch (req->h.cmd)
        {
            case CMD_CFG_READ_REQ:
                DEBUG_PR_INFO("CFG_READ_REQ");
                cfg_read_req(req, config_if_rx_size);
                break;

            case CMD_CFG_WRITE_REQ:
                DEBUG_PR_INFO("CFG_WRITE_REQ");
                cfg_write_req(req, config_if_rx_size);
                break;

            case CMD_CFG_SAVE_REQ:
                DEBUG_PR_INFO("CFG_SAVE_REQ");
                cfg_save_req(req, config_if_rx_size);
                break;

            case CMD_CFG_RESTORE_REQ:
                DEBUG_PR_INFO("CFG_RESTORE_REQ");
                cfg_restore_req(req, config_if_rx_size);
                break;

            case CMD_CFG_ERASE_REQ:
                DEBUG_PR_INFO("CFG_ERASE_REQ");
                cfg_erase_req(req, config_if_rx_size);
                break;

            case CMD_CFG_PROTECT_REQ:
                DEBUG_PR_INFO("CFG_PROTECT_REQ");
                cfg_protect_req(req, config_if_rx_size);
                break;

            case CMD_CFG_UNPROTECT_REQ:
                DEBUG_PR_INFO("CFG_UNPROTECT_REQ");
                cfg_unprotect_req(req, config_if_rx_size);
                break;

            case CMD_GPS_WRITE_REQ:
                DEBUG_PR_INFO("GPS_WRITE_REQ");
                gps_write_req(req, config_if_rx_size);
                break;

            case CMD_GPS_READ_REQ:
                DEBUG_PR_INFO("GPS_READ_REQ");
                gps_read_req(req, config_if_rx_size);
                break;

            case CMD_GPS_CONFIG_REQ:
                DEBUG_PR_INFO("GPS_CONFIG_REQ");
                gps_config_req(req, config_if_rx_size);
                break;

            case CMD_BLE_CONFIG_REQ:
                DEBUG_PR_INFO("BLE_CONFIG_REQ");
                ble_config_req(req, config_if_rx_size);
                break;

            case CMD_BLE_WRITE_REQ:
                DEBUG_PR_INFO("BLE_WRITE_REQ");
                ble_write_req(req, config_if_rx_size);
                break;

            case CMD_BLE_READ_REQ:
                DEBUG_PR_INFO("BLE_READ_REQ");
                ble_read_req(req, config_if_rx_size);
                break;

            case CMD_STATUS_REQ:
                DEBUG_PR_INFO("STATUS_REQ");
                status_req(req, config_if_rx_size);
                break;

            case CMD_FW_SEND_IMAGE_REQ:
                DEBUG_PR_INFO("FW_SEND_IMAGE_REQ");
                fw_send_image_req(req, config_if_rx_size);
                break;

            case CMD_FW_APPLY_IMAGE_REQ:
                DEBUG_PR_INFO("FW_APPLY_IMAGE_REQ");
                fw_apply_image_req(req, config_if_rx_size);
                break;

            case CMD_RESET_REQ:
                DEBUG_PR_INFO("RESET_REQ");
                reset_req(req, config_if_rx_size);
                break;

            case CMD_BATTERY_STATUS_REQ:
                DEBUG_PR_INFO("BATTERY_STATUS_REQ");
                battery_status_req(req, config_if_rx_size);
                break;

            case CMD_LOG_CREATE_REQ:
                DEBUG_PR_INFO("LOG_CREATE_REQ");
                log_create_req(req, config_if_rx_size);
                break;

            case CMD_LOG_ERASE_REQ:
                DEBUG_PR_INFO("LOG_ERASE_REQ");
                log_erase_req(req, config_if_rx_size);
                break;

            case CMD_LOG_READ_REQ:
                DEBUG_PR_INFO("LOG_READ_REQ");
                log_read_req(req, config_if_rx_size);
                break;

            default:
                DEBUG_PR_WARN("Unhandled command: id %d", req->h.cmd);
                // Don't return an error. Fail silent
                break;
        }

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
    uint16_t last_index = 0;
    int tag = 0; // The first tag - WARN: this relies on there being a tag ID of 0x0000
    int return_code;
    bool tag_not_set = false;
    do
    {
        return_code = sys_config_get(tag, NULL);
        if (SYS_CONFIG_ERROR_TAG_NOT_SET == return_code)
        {
            DEBUG_PR_WARN("Tag 0x%04X not set", tag);
            tag_not_set = true;
        }
        tag = sys_config_iterate(tag, &last_index);
    }
    while (SYS_CONFIG_ERROR_NO_MORE_TAGS != tag);

    if (tag_not_set)
        sm_set_state(SM_STATE_STANDBY_PROVISIONING_NEEDED);
    else
        sm_set_state(SM_STATE_OPERATIONAL);


    //sm_set_state(SM_STATE_STANDBY_PROVISIONING_NEEDED);

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

    // Queue a receive request
    config_if_receive(rx_buffer, sizeof(rx_buffer));

    handle_config_if_requests();

    //parse_rx_buffer();

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

}

void operational_state(void)
{
//    if (syshal_usb_available())
//    {
//        DEBUG_PR_INFO("USB message received");
//        uint8_t buffer[100];
//        uint32_t size = syshal_usb_receive(buffer, sizeof(buffer));
//        syshal_usb_transfer(buffer, size);
//    }
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// STATE HANDLERS ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void exception_handler(CEXCEPTION_T e)
{

    switch (e)
    {
        case EXCEPTION_REQ_WRONG_SIZE:
            DEBUG_PR_ERROR("EXCEPTION_REQ_WRONG_SIZE");
            config_if_rx_pending = false; // This req message is erroneous so ignore it
            break;

        case EXCEPTION_RESP_TX_PENDING:
            // We're already transmitted a response
            // This is thrown to avoid overwritting the TX buffer before it is sent
            DEBUG_PR_ERROR("EXCEPTION_RESP_TX_PENDING");
            break;

        case EXCEPTION_TX_BUFFER_FULL:
            DEBUG_PR_ERROR("EXCEPTION_TX_BUFFER_FULL");
            break;

        default:
            DEBUG_PR_ERROR("Unknown exception");
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
        exception_handler(e);
    }
}