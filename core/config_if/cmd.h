/* cmd.h - Configuration interface commands
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

#ifndef _CMD_H_
#define _CMD_H_

#define CMD_MAX_PAYLOAD     ((20 * 1024) - sizeof(cmd_hdr_t))
#define CMD_SYNCWORD        0x55555555
#define CMD_MAX_BUFFER      (16 * 1024)

#define CMD_MIN_SIZE        sizeof(cmd_hdr_t)
#define CMD_MAX_SIZE        (20 * 1024)

#define CMD_SET_HDR(p, i, l)  \
    p->h.sync  = CMD_SYNCWORD; \
    p->h.cmd   = i;

typedef struct
{
    uint32_t sync; // Start of command synchronization byte
    uint32_t cmd;
} cmd_hdr_t;

typedef enum
{
    CMD_GENERIC_RESP, // Generic response message sent where only an error response is needed

    ///////////////// Configuration /////////////////
    CMD_CFG_READ_REQ,      // Read single configuration tag or all configuration tags
    CMD_CFG_WRITE_REQ,     // Write new configuration items as a series of tag/value pairs
    CMD_CFG_SAVE_REQ,      // Save all current configuration settings to flash memory
    CMD_CFG_RESTORE_REQ,   // Restore all current configuration settings from flash memory
    CMD_CFG_ERASE_REQ,     // Erase a single or all configuration tags in flash memory
    CMD_CFG_PROTECT_REQ,   // Protect the configuration file in flash memory
    CMD_CFG_UNPROTECT_REQ, // Unprotect the configuration file in flash memory
    CMD_CFG_READ_RESP,

    ////////////////// GPS Bridge ///////////////////
    CMD_GPS_WRITE_REQ,      // Send UBX commands directly to the GPS module
    CMD_GPS_READ_REQ,       // Receive UBX command responses directly from the GPS module
    CMD_GPS_RESP,           // The response from the GPS module
    CMD_GPS_IRQ_CONFIG_REQ, // Allow a GPS IRQ events to be generated and sent over the USB interrupt endpoint.  This shall be used to indicate that data is available to be read from the internal FIFO

    ////////////////// BLE Bridge ///////////////////
    CMD_BLE_IRQ_CONFIG_REQ, // Allow BLE IRQ events to be generated and sent over the USB interrupt endpoint
    CMD_BLE_WRITE_REQ,      // Initiate a write to the BLE module at Address with data of Length
    CMD_BLE_READ_REQ,       // Initiate a read from the BLE module from Address for data of Length

    //////////////////// System /////////////////////
    CMD_STATUS_REQ,                 // Request firmware status
    CMD_STATUS_RESP,                // Firmware status response
    CMD_FW_SEND_IMAGE_REQ,          // Request to send a new firmware imag and store temporarily in local flash memory
    CMD_FW_SEND_IMAGE_COMPLETE_IND, // This shall be sent by the server to the client to indicate all bytes have been received and stored
    CMD_FW_APPLY_IMAGE_REQ,         // Request to apply an existing firmware image in temporary storage to the target
    CMD_RESET_REQ,                  // Request to reset the system

    //////////////////// Battery /////////////////////
    BATTERY_STATUS_REQ,  // Request battery status
    BATTERY_STATUS_RESP, // Error response carrying the battery status information

    //////////////////// Logging /////////////////////
    LOG_CREATE_REQ, // Request to create a log file of the specified operation mode and length
    LOG_ERASE_REQ,  // Request to erase the current log file
    LOG_READ_REQ,   // Read the log file from the starting offset for the given number of bytes
    LOG_READ_RESP,  // Response for a read request

} cmd_id_t;

// Generic response message
typedef struct
{
    uint8_t error_code;
} cmd_generic_resp_t;

///////////////// Configuration /////////////////
typedef struct
{
    uint16_t configuration_tag;
} cmd_cfg_read_req_t;

typedef struct
{
    uint32_t length;
} cmd_cfg_write_req_t;

typedef struct
{
    uint16_t configuration_tag;
} cmd_cfg_erase_req_t;

typedef struct
{
    uint8_t error_code;
    uint32_t length
} cmd_cfg_read_resp_t;

////////////////// GPS Bridge ///////////////////
typedef struct
{
    uint32_t length;
} cmd_gps_write_req_t;

typedef struct
{
    uint32_t length;
} cmd_gps_read_req_t;

typedef struct
{
    uint8_t error_code;
    uint32_t length;
} cmd_gps_resp_t;

typedef struct
{
    uint8_t enable;
} cmd_gps_irq_config_req_t;

////////////////// BLE Bridge ///////////////////
typedef struct
{
    uint8_t enable;
} cmd_ble_irq_config_req_t;

typedef struct
{
    uint8_t address;
    uint16_t length;
} cmd_ble_write_req_t;

typedef struct
{
    uint8_t address;
    uint16_t length;
} cmd_ble_read_req_t;

//////////////////// System /////////////////////
typedef struct
{
    uint8_t error_code;
    uint32_t firmware_version;
    uint32_t firmware_checksum;
    uint32_t configuration_format_version;
} cmd_status_resp_t;

typedef struct
{
    uint8_t image_type;
    uint32_t length;
    uint32_t CRC;
} cmd_fw_send_image_req_t;

typedef struct
{
    uint8_t error_code;
} cmd_fw_send_image_complete_ind_t;

typedef struct
{
    uint8_t image_type;
} cmd_fw_apply_image_req_t;

typedef struct
{
    uint8_t reset_type;
} cmd_reset_req_t;

//////////////////// Battery /////////////////////
typedef struct
{
    uint8_t error_code;
    uint8_t charging_indicator;
    uint8_t charge_level; // Charge level in percent
} cmd_battery_status_resp_t;

//////////////////// Logging /////////////////////
typedef struct
{
    uint8_t mode;
    uint8_t sync_enable;
    uint8_t max_file_size;
} cmd_log_create_req_t;

typedef struct
{
    uint32_t start_offset;
    uint32_t length;
} cmd_log_read_req_t;

typedef struct
{
    uint8_t error_code;
    uint32_t length;
} cmd_log_read_resp_t;

typedef struct __attribute__((__packed__))
{
    cmd_hdr_t h;
    union
    {
        uint8_t                             bytes[CMD_MAX_PAYLOAD];
        cmd_generic_resp_t                  generic_resp;
        cmd_cfg_read_req_t                  cfg_read_req;
        cmd_cfg_write_req_t                 cfg_write_req;
        cmd_cfg_erase_req_t                 cfg_erase_req;
        cmd_cfg_read_resp_t                 cfg_read_resp;
        cmd_gps_write_req_t                 gps_write_req;
        cmd_gps_read_req_t                  gps_read_req;
        cmd_gps_resp_t                      gps_resp;
        cmd_gps_irq_config_req_t            gps_irq_config_req;
        cmd_ble_irq_config_req_t            ble_irq_config_req;
        cmd_ble_write_req_t                 ble_write_req;
        cmd_ble_read_req_t                  ble_read_req;
        cmd_status_resp_t                   status_resp;
        cmd_fw_send_image_req_t             fw_send_image_req;
        cmd_fw_send_image_complete_ind_t    fw_send_image_complete_ind;
        cmd_fw_apply_image_req_t            fw_apply_image_req;
        cmd_reset_req_t                     reset_req;
        cmd_battery_status_resp_t           battery_status_resp;
        cmd_log_create_req_t                log_create_req;
        cmd_log_read_req_t                  log_read_req;
        cmd_log_read_resp_t                 log_read_resp;
    } p;
} cmd_t;

#endif /* _CMD_H_ */
