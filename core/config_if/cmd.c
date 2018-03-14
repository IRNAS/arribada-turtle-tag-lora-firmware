#include "cmd.h"

uint32_t cmd_size_of_command(cmd_id_t command)
{

    uint32_t expectedSize = sizeof(cmd_hdr_t);

    switch (command)
    {

        case CMD_CFG_READ_REQ:
            expectedSize += sizeof(cmd_cfg_read_req_t);
            break;

        case CMD_CFG_WRITE_REQ:
            expectedSize += sizeof(cmd_cfg_write_req_t);
            break;

        case CMD_CFG_ERASE_REQ:
            expectedSize += sizeof(cmd_cfg_erase_req_t);
            break;

        case CMD_CFG_WRITE_CNF:
            expectedSize += sizeof(cmd_cfg_write_cnf_t);
            break;

        case CMD_GPS_WRITE_REQ:
            expectedSize += sizeof(cmd_gps_write_req_t);
            break;

        case CMD_GPS_READ_REQ:
            expectedSize += sizeof(cmd_gps_read_req_t);
            break;

        case CMD_GPS_CONFIG_REQ:
            expectedSize += sizeof(cmd_gps_config_req_t);
            break;

        case CMD_BLE_CONFIG_REQ:
            expectedSize += sizeof(cmd_ble_config_req_t);
            break;

        case CMD_BLE_WRITE_REQ:
            expectedSize += sizeof(cmd_ble_write_req_t);
            break;

        case CMD_BLE_READ_REQ:
            expectedSize += sizeof(cmd_ble_read_req_t);
            break;

        case CMD_STATUS_RESP:
            expectedSize += sizeof(cmd_status_resp_t);
            break;

        case CMD_FW_SEND_IMAGE_REQ:
            expectedSize += sizeof(cmd_fw_send_image_req_t);
            break;

        case CMD_FW_SEND_IMAGE_COMPLETE_CNF:
            expectedSize += sizeof(cmd_fw_send_image_complete_cnf_t);
            break;

        case CMD_FW_APPLY_IMAGE_REQ:
            expectedSize += sizeof(cmd_fw_apply_image_req_t);
            break;

        case CMD_RESET_REQ:
            expectedSize += sizeof(cmd_reset_req_t);
            break;

        case CMD_LOG_CREATE_REQ:
            expectedSize += sizeof(cmd_log_create_req_t);
            break;

        case CMD_LOG_READ_REQ:
            expectedSize += sizeof(cmd_log_read_req_t);
            break;

        default:
        break;
    }

    return expectedSize;
}