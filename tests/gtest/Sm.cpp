// Fs.cpp - Filesystem unit tests
//
// Copyright (C) 2018 Arribada
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
extern "C" {
#include "unity.h"
#include <assert.h>
#include <stdint.h>
#include "Mocksyshal_batt.h"
#include "Mocksyshal_gpio.h"
#include "Mocksyshal_uart.h"
#include "Mocksyshal_spi.h"
#include "Mocksyshal_i2c.h"
#include "Mocksyshal_time.h"
#include "Mocksyshal_flash.h"
#include "Mockconfig_if.h"
#include "fs_priv.h"
#include "fs.h"
#include "cmd.h"
#include "sys_config.h"
#include "sm.h"
#include <stdlib.h>
}

#include "googletest.h"

#include <cstdlib>
#include <cstring>
#include <iostream>

#include <utility>
#include <queue>
#include <vector>

#define FS_FILE_ID_CONF             (0) // The File ID of the configuration data
#define FS_FILE_ID_STM32_IMAGE      (1) // STM32 application image
#define FS_FILE_ID_BLE_APP_IMAGE    (2) // BLE application image
#define FS_FILE_ID_BLE_SOFT_IMAGE   (3) // BLE soft-device image
#define FS_FILE_ID_LOG              (4) // Sensor log file

#define FLASH_SIZE          (FS_PRIV_SECTOR_SIZE * FS_PRIV_MAX_SECTORS)
#define ASCII(x)            ((x) >= 32 && (x) <= 127) ? (x) : '.'

static bool fs_trace;
char flash_ram[FLASH_SIZE];

// Dummy functions, used to ignore all calls to this function
void syshal_batt_init_dummy(uint32_t instance, int cmock_num_calls) {}
void syshal_gpio_init_dummy(uint32_t pin, int cmock_num_calls) {}
int syshal_uart_init_dummy(uint32_t instance, int cmock_num_calls) {return SYSHAL_UART_NO_ERROR;}
int syshal_spi_init_dummy(uint32_t instance, int cmock_num_calls) {return SYSHAL_SPI_NO_ERROR;}
int syshal_i2c_init_dummy(uint32_t instance, int cmock_num_calls) {return SYSHAL_I2C_NO_ERROR;}
int config_if_init_dummy(config_if_backend_t backend, int cmock_num_calls) {return CONFIG_IF_NO_ERROR;}

// syshal_time_get_ticks_ms callback function
uint32_t syshal_time_get_ticks_ms_value;
uint32_t syshal_time_get_ticks_ms_callback(int cmock_num_calls)
{
    return syshal_time_get_ticks_ms_value;
}

// config_if_receive callback function
uint8_t * config_if_receive_buffer = NULL;
int config_if_receive_callback(uint8_t * data, uint32_t size, int cmock_num_calls)
{
    config_if_receive_buffer = data;
    return CONFIG_IF_NO_ERROR;
}

// config_if_send callback function
uint8_t * config_if_send_buffer = NULL;
uint32_t config_if_send_size;
int config_if_send_callback(uint8_t * data, uint32_t size, int cmock_num_calls)
{
    config_if_send_buffer = data;
    config_if_send_size = size;
    return CONFIG_IF_NO_ERROR;
}

// Syshal_flash callbacks
int syshal_flash_read_Callback(uint32_t device, void * dest, uint32_t address, uint32_t size, int cmock_num_calls)
{
    //printf("syshal_flash_read(%08x,%u)\n", address, size);
    for (unsigned int i = 0; i < size; i++)
        ((char *)dest)[i] = flash_ram[address + i];

    return 0;
}

int syshal_flash_write_Callback(uint32_t device, const void * src, uint32_t address, uint32_t size, int cmock_num_calls)
{
    if (fs_trace)
        printf("syshal_flash_write(%08x, %u)\n", address, size);
    for (unsigned int i = 0; i < size; i++)
    {
        /* Ensure no new bits are being set */
        if ((((char *)src)[i] & flash_ram[address + i]) ^ ((char *)src)[i])
        {
            printf("syshal_flash_write: Can't set bits from 0 to 1 (%08x: %02x => %02x)\n", address + i,
                   (uint8_t)flash_ram[address + i], (uint8_t)((char *)src)[i]);
            assert(0);
        }
        flash_ram[address + i] = ((char *)src)[i];
    }

    return 0;
}

int syshal_flash_erase_Callback(uint32_t device, uint32_t address, uint32_t size, int cmock_num_calls)
{
    /* Make sure address is sector aligned */
    if (address % FS_PRIV_SECTOR_SIZE || size % FS_PRIV_SECTOR_SIZE)
    {
        printf("syshal_flash_erase: Non-aligned address %08x", address);
        assert(0);
    }

    for (unsigned int i = 0; i < size; i++)
        flash_ram[address + i] = 0xFF;

    return 0;
}

class SmTest : public ::testing::Test
{

    virtual void SetUp()
    {
        Mocksyshal_batt_Init();
        Mocksyshal_gpio_Init();
        Mocksyshal_uart_Init();
        Mocksyshal_spi_Init();
        Mocksyshal_i2c_Init();
        Mockconfig_if_Init();
        Mocksyshal_flash_Init();

        // Callbacks
        syshal_flash_read_StubWithCallback(syshal_flash_read_Callback);
        syshal_flash_write_StubWithCallback(syshal_flash_write_Callback);
        syshal_flash_erase_StubWithCallback(syshal_flash_erase_Callback);

        syshal_time_get_ticks_ms_StubWithCallback(syshal_time_get_ticks_ms_callback);
        config_if_receive_StubWithCallback(config_if_receive_callback);
        config_if_send_StubWithCallback(config_if_send_callback);

        // Clear FLASH contents
        for (unsigned int i = 0; i < FLASH_SIZE; i++)
            flash_ram[i] = 0xFF;

        fs_trace = false; // turn FS trace off

        // Clear all configuration tags
        clear_all_configuration_tags_RAM();

        // Set global variables to defaults
        config_if_receive_buffer = NULL;
        config_if_send_buffer = NULL;
        config_if_send_size = 0;
    }

    virtual void TearDown()
    {
        // Cleanup Mocks
        Mocksyshal_batt_Verify();
        Mocksyshal_batt_Destroy();
        Mocksyshal_gpio_Verify();
        Mocksyshal_gpio_Destroy();
        Mocksyshal_uart_Verify();
        Mocksyshal_uart_Destroy();
        Mocksyshal_spi_Verify();
        Mocksyshal_spi_Destroy();
        Mocksyshal_i2c_Verify();
        Mocksyshal_i2c_Destroy();
        Mockconfig_if_Verify();
        Mockconfig_if_Destroy();
        Mocksyshal_flash_Verify();
        Mocksyshal_flash_Destroy();
    }

public:

    // Functions
    void HardwareInit()
    {
        // Ignore all calls to these functions
        syshal_batt_init_StubWithCallback(syshal_batt_init_dummy);
        syshal_gpio_init_StubWithCallback(syshal_gpio_init_dummy);
        syshal_uart_init_StubWithCallback(syshal_uart_init_dummy);
        syshal_spi_init_StubWithCallback(syshal_spi_init_dummy);
        syshal_i2c_init_StubWithCallback(syshal_i2c_init_dummy);
        config_if_init_StubWithCallback(config_if_init_dummy);
    }

    void set_all_configuration_tags_RAM()
    {
        uint8_t empty_buffer[SYS_CONFIG_MAX_DATA_SIZE];

        sys_config_set(SYS_CONFIG_TAG_GPS_LOG_POSITION_ENABLE, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_gps_log_position_enable_t));
        sys_config_set(SYS_CONFIG_TAG_GPS_LOG_TTFF_ENABLE, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_gps_log_ttff_enable_t));
        sys_config_set(SYS_CONFIG_TAG_GPS_TRIGGER_MODE, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_gps_trigger_mode_t));
        sys_config_set(SYS_CONFIG_TAG_GPS_UART_BAUD_RATE, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_gps_uart_baud_rate_t));
        sys_config_set(SYS_CONFIG_TAG_RTC_SYNC_TO_GPS_ENABLE, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_rtc_sync_to_gps_enable_t));
        sys_config_set(SYS_CONFIG_TAG_RTC_CURRENT_DATE_AND_TIME, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_rtc_current_date_and_time_t));
        sys_config_set(SYS_CONFIG_TAG_LOGGING_ENABLE, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_enable_t));
        sys_config_set(SYS_CONFIG_TAG_LOGGING_BYTES_WRITTEN, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_bytes_written_t));
        sys_config_set(SYS_CONFIG_TAG_LOGGING_FILE_SIZE, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_file_size_t));
        sys_config_set(SYS_CONFIG_TAG_LOGGING_FILE_TYPE, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_file_type_t));
        sys_config_set(SYS_CONFIG_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_group_sensor_readings_enable_t));
        sys_config_set(SYS_CONFIG_TAG_LOGGING_START_END_SYNC_ENABLE, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_start_end_sync_enable_t));
        sys_config_set(SYS_CONFIG_TAG_LOGGING_DATE_TIME_STAMP_ENABLE, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_date_time_stamp_enable_t));
        sys_config_set(SYS_CONFIG_TAG_LOGGING_HIGH_RESOLUTION_TIMER_ENABLE, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_high_resolution_timer_enable_t));
        sys_config_set(SYS_CONFIG_TAG_AXL_LOG_ENABLE, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_axl_log_enable_t));
        sys_config_set(SYS_CONFIG_TAG_AXL_CONFIG, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_axl_config_t));
        sys_config_set(SYS_CONFIG_TAG_AXL_G_FORCE_HIGH_THRESHOLD, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_axl_g_force_high_threshold_t));
        sys_config_set(SYS_CONFIG_TAG_AXL_SAMPLE_RATE, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_axl_sample_rate_t));
        sys_config_set(SYS_CONFIG_TAG_AXL_MODE, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_axl_mode_t));
        sys_config_set(SYS_CONFIG_TAG_PRESSURE_SENSOR_LOG_ENABLE, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_pressure_sensor_log_enable_t));
        sys_config_set(SYS_CONFIG_TAG_PRESSURE_SAMPLE_RATE, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_pressure_sample_rate_t));
        sys_config_set(SYS_CONFIG_TAG_PRESSURE_LOW_THRESHOLD, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_pressure_low_threshold_t));
        sys_config_set(SYS_CONFIG_TAG_PRESSURE_HIGH_THRESHOLD, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_pressure_high_threshold_t));
        sys_config_set(SYS_CONFIG_TAG_PRESSURE_MODE, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_pressure_mode_t));
        sys_config_set(SYS_CONFIG_TAG_TEMP_SENSOR_LOG_ENABLE, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_temp_sensor_log_enable_t));
        sys_config_set(SYS_CONFIG_TAG_TEMP_SENSOR_SAMPLE_RATE, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_temp_sensor_sample_rate_t));
        sys_config_set(SYS_CONFIG_TAG_TEMP_SENSOR_LOW_THRESHOLD, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_temp_sensor_low_threshold_t));
        sys_config_set(SYS_CONFIG_TAG_TEMP_SENSOR_HIGH_THRESHOLD, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_temp_sensor_high_threshold_t));
        sys_config_set(SYS_CONFIG_TAG_TEMP_SENSOR_MODE, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_temp_sensor_mode_t));
        sys_config_set(SYS_CONFIG_TAG_SYSTEM_DEVICE_IDENTIFIER, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_system_device_identifier_t));
        sys_config_set(SYS_CONFIG_TAG_BLUETOOTH_UUID, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_bluetooth_uuid_t));
        sys_config_set(SYS_CONFIG_TAG_BLUETOOTH_BEACON_ENABLE, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_bluetooth_beacon_enable_t));
        sys_config_set(SYS_CONFIG_TAG_BLUETOOTH_BEACON_GEO_FENCE_TRIGGER_LOCATION, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_bluetooth_beacon_geo_fence_trigger_location_t));
        sys_config_set(SYS_CONFIG_TAG_BLUETOOTH_BEACON_ADVERTISING_INTERVAL, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_bluetooth_beacon_advertising_interval_t));
        sys_config_set(SYS_CONFIG_TAG_BLUETOOTH_BEACON_ADVERTISING_CONFIGURATION, &empty_buffer, SYS_CONFIG_TAG_DATA_SIZE(sys_config_bluetooth_beacon_advertising_configuration_t));
    }

    void clear_all_configuration_tags_RAM()
    {
        // Clear all configuration tags
        uint16_t last_index = 0;
        uint16_t tag;
        while (!sys_config_iterate(&tag, &last_index))
        {
            sys_config_unset(tag);
        }
    }

    // Send config_if message to the state machine
    void send_message(cmd_t message, uint32_t size)
    {
        // Copy message into receive buffer
        memcpy(config_if_receive_buffer, &message, size);

        // Generate receive request event
        config_if_event_t event;
        event.id = CONFIG_IF_EVENT_RECEIVE_COMPLETE;
        event.receive.buffer = config_if_receive_buffer;
        event.receive.size = size;
        config_if_event_handler(&event);
    }

    // Receive config_if message from the state machine
    cmd_t receive_message(void)
    {
        // Copy message into receive buffer
        cmd_t message;
        memcpy(&message, config_if_send_buffer, config_if_send_size);

        // Generate receive request event
        config_if_event_t event;
        event.id = CONFIG_IF_EVENT_SEND_COMPLETE;
        event.receive.buffer = config_if_send_buffer;
        event.receive.size = config_if_send_size;
        config_if_event_handler(&event);

        return message;
    }

    // Create a config_if connection event
    void connect(void)
    {
        config_if_event_t event;
        event.id = CONFIG_IF_EVENT_CONNECTED;
        config_if_event_handler(&event);
    }

    // Create a config_if disconnect event
    void disconnect(void)
    {
        config_if_event_t event;
        event.id = CONFIG_IF_EVENT_DISCONNECTED;
        config_if_event_handler(&event);
    }

    void startup_provisioning_needed(void)
    {
        sm_set_state(SM_STATE_BOOT);
        HardwareInit();

        syshal_batt_charging_ExpectAndReturn(false);
        syshal_batt_state_ExpectAndReturn(POWER_SUPPLY_CAPACITY_LEVEL_FULL);

        syshal_gpio_set_output_high_Expect(GPIO_LED3); // Status LED

        sm_iterate();

        EXPECT_EQ(SM_STATE_STANDBY_PROVISIONING_NEEDED, sm_get_state());
    }

};

TEST_F(SmTest, StateSet)
{
    sm_set_state(SM_STATE_BOOT);
    EXPECT_EQ(SM_STATE_BOOT, sm_get_state());
}

TEST_F(SmTest, BootConfigurationDataDoesNotExist)
{
    sm_set_state(SM_STATE_BOOT);
    HardwareInit();

    syshal_batt_charging_ExpectAndReturn(false);
    syshal_batt_state_ExpectAndReturn(POWER_SUPPLY_CAPACITY_LEVEL_FULL);

    syshal_gpio_set_output_high_Expect(GPIO_LED3); // Status LED

    sm_iterate();

    EXPECT_EQ(SM_STATE_STANDBY_PROVISIONING_NEEDED, sm_get_state());
}

TEST_F(SmTest, BootConfigurationDataExistsButIncomplete)
{
    sm_set_state(SM_STATE_BOOT);
    HardwareInit();

    syshal_batt_charging_ExpectAndReturn(false);
    syshal_batt_state_ExpectAndReturn(POWER_SUPPLY_CAPACITY_LEVEL_FULL);

    syshal_gpio_set_output_high_Expect(GPIO_LED3); // Status LED

    sm_iterate();

    EXPECT_EQ(SM_STATE_STANDBY_PROVISIONING_NEEDED, sm_get_state());
}

TEST_F(SmTest, BootBatteryCharging)
{
    sm_set_state(SM_STATE_BOOT);
    HardwareInit();

    syshal_batt_charging_ExpectAndReturn(true);

    sm_iterate();

    EXPECT_EQ(SM_STATE_STANDBY_BATTERY_CHARGING, sm_get_state());
}

TEST_F(SmTest, BootBatteryLevelLow)
{
    sm_set_state(SM_STATE_BOOT);
    HardwareInit();

    syshal_batt_charging_ExpectAndReturn(false);
    syshal_batt_state_ExpectAndReturn(POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL);

    sm_iterate();

    EXPECT_EQ(SM_STATE_STANDBY_BATTERY_LEVEL_LOW, sm_get_state());
}

TEST_F(SmTest, BootConfigurationComplete)
{
    sm_set_state(SM_STATE_BOOT);
    HardwareInit();

    syshal_batt_charging_ExpectAndReturn(false);
    syshal_batt_state_ExpectAndReturn(POWER_SUPPLY_CAPACITY_LEVEL_FULL);

    syshal_gpio_set_output_high_Expect(GPIO_LED3); // Status LED

    // Set all the configuration tags in RAM
    set_all_configuration_tags_RAM();

    // Store all the tags into FLASH
    fs_t file_system;
    fs_handle_t file_system_handle;
    uint32_t bytes_written;

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_format(file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_open(file_system, &file_system_handle, FS_FILE_ID_CONF, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(file_system_handle, &sys_config, sizeof(sys_config), &bytes_written));
    EXPECT_EQ(FS_NO_ERROR, fs_close(file_system_handle));

    // Clear all tags in RAM
    clear_all_configuration_tags_RAM();

    sm_iterate();

    EXPECT_EQ(SM_STATE_OPERATIONAL, sm_get_state());
}

TEST_F(SmTest, ProvisioningNeededState)
{
    startup_provisioning_needed(); // Boot and transition to provisioning needed state

    connect(); // Connect the config_if

    sm_iterate();

    EXPECT_EQ(SM_STATE_PROVISIONING, sm_get_state());
}

TEST_F(SmTest, ProvisioningDisconnect)
{
    startup_provisioning_needed(); // Boot and transition to provisioning needed state

    connect(); // Connect the config_if

    sm_iterate();

    EXPECT_EQ(SM_STATE_PROVISIONING, sm_get_state());

    config_if_receive_IgnoreAndReturn(CONFIG_IF_NO_ERROR);

    disconnect(); // Disconnect the config_if

    sm_iterate();

    EXPECT_EQ(SM_STATE_STANDBY_PROVISIONING_NEEDED, sm_get_state());
}

TEST_F(SmTest, StatusRequest)
{
    startup_provisioning_needed(); // Boot and transition to provisioning needed state

    connect(); // Connect the config_if

    sm_iterate();

    EXPECT_EQ(SM_STATE_PROVISIONING, sm_get_state());

    sm_iterate(); // Queue the first receive

    // Generate reset request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_STATUS_REQ);
    send_message(req, CMD_SIZE_HDR);

    sm_iterate(); // Process the message

    // Check the response
    cmd_t resp = receive_message();
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_STATUS_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_status_resp.error_code);
    EXPECT_EQ(0, resp.p.cmd_status_resp.stm_firmware_version);
    EXPECT_EQ(0, resp.p.cmd_status_resp.ble_firmware_version);
    EXPECT_EQ(0, resp.p.cmd_status_resp.configuration_format_version);
}

TEST_F(SmTest, CfgWriteOne)
{
    startup_provisioning_needed(); // Boot and transition to provisioning needed state

    connect(); // Connect the config_if

    sm_iterate();

    EXPECT_EQ(SM_STATE_PROVISIONING, sm_get_state());

    sm_iterate(); // Queue the first receive

    // Generate cfg write request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_WRITE_REQ);
    req.p.cmd_cfg_write_req.length = SYS_CONFIG_TAG_ID_SIZE + SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_group_sensor_readings_enable_t);
    send_message(req, CMD_SIZE(cmd_cfg_write_req_t));

    sm_iterate(); // Process the message

    // Check the response
    cmd_t resp;
    resp = receive_message();
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);

    // Generate cfg tag data packet
    uint8_t tag_data_packet[3];
    tag_data_packet[0] = uint16_t(SYS_CONFIG_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE) & 0x00FF;
    tag_data_packet[1] = (uint16_t(SYS_CONFIG_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE) & 0xFF00) >> 8;
    tag_data_packet[2] = true; // Enable

    cmd_t * message = (cmd_t *) &tag_data_packet;
    send_message(*message, sizeof(tag_data_packet));

    sm_iterate(); // Process the message

    // Check the response
    resp = receive_message();
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_CFG_WRITE_CNF, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_cfg_write_cnf.error_code);

    // Check the tag was correctly set
    EXPECT_TRUE(sys_config.sys_config_logging_group_sensor_readings_enable.contents.enable);
}

TEST_F(SmTest, CfgWriteOneInvalidTag)
{
    startup_provisioning_needed(); // Boot and transition to provisioning needed state

    connect(); // Connect the config_if

    sm_iterate();

    EXPECT_EQ(SM_STATE_PROVISIONING, sm_get_state());

    sm_iterate(); // Queue the first receive

    // Generate cfg write request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_WRITE_REQ);
    req.p.cmd_cfg_write_req.length = SYS_CONFIG_TAG_ID_SIZE + SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_group_sensor_readings_enable_t);
    send_message(req, CMD_SIZE(cmd_cfg_write_req_t));

    sm_iterate(); // Process the message

    // Check the response
    cmd_t resp;
    resp = receive_message();
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);

    // Generate cfg tag data packet
    uint8_t tag_data_packet[3];
    uint16_t invalid_tag_ID = 0xABAB;
    tag_data_packet[0] = invalid_tag_ID & 0x00FF;
    tag_data_packet[1] = (invalid_tag_ID & 0xFF00) >> 8;
    tag_data_packet[2] = true; // Enable

    cmd_t * message = (cmd_t *) &tag_data_packet;
    send_message(*message, sizeof(tag_data_packet));

    sm_iterate(); // Process the message
    sm_iterate(); // Generate the error response

    // Check the response
    resp = receive_message();
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_CFG_WRITE_CNF, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_INVALID_CONFIG_TAG, resp.p.cmd_cfg_write_cnf.error_code);
}

TEST_F(SmTest, CfgReadOne)
{
    // Set a tag up for reading later
    sys_config.sys_config_logging_group_sensor_readings_enable.contents.enable = true;
    sys_config.sys_config_logging_group_sensor_readings_enable.hdr.set = true;

    startup_provisioning_needed(); // Boot and transition to provisioning needed state

    connect(); // Connect the config_if

    sm_iterate();

    EXPECT_EQ(SM_STATE_PROVISIONING, sm_get_state());

    sm_iterate(); // Queue the first receive

    // Generate cfg read request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_READ_REQ);
    req.p.cmd_cfg_read_req.configuration_tag = SYS_CONFIG_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE;
    send_message(req, CMD_SIZE(cmd_cfg_read_req_t));

    sm_iterate(); // Process the message

    // Check the response
    cmd_t resp;
    resp = receive_message();
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_CFG_READ_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_cfg_read_resp.error_code);
    uint32_t expected_length = resp.p.cmd_cfg_read_resp.length;

    sm_iterate(); // Send the data packet
    resp = receive_message();
    uint8_t * message = (uint8_t *) &resp;

    uint16_t tag = 0;
    tag |= (uint16_t) message[0] & 0x00FF;
    tag |= (uint16_t) (message[1] << 8) & 0xFF00;
    EXPECT_EQ(SYS_CONFIG_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE, tag);

    // Check the tag was correctly read
    EXPECT_TRUE(message[2]);
}

TEST_F(SmTest, CfgSaveSuccess)
{
    startup_provisioning_needed(); // Boot and transition to provisioning needed state

    connect(); // Connect the config_if

    sm_iterate();

    EXPECT_EQ(SM_STATE_PROVISIONING, sm_get_state());

    sm_iterate(); // Queue the first receive

    // Generate cfg save request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_SAVE_REQ);
    send_message(req, CMD_SIZE_HDR);

    sm_iterate(); // Process the message

    // Check the response
    cmd_t resp;
    resp = receive_message();
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);

    // Check the configuration file in FLASH matches the configuration in RAM
    fs_t file_system;
    fs_handle_t file_system_handle;
    uint32_t bytes_read;
    uint8_t flash_config_data[sizeof(sys_config)];

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_open(file_system, &file_system_handle, FS_FILE_ID_CONF, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_read(file_system_handle, &flash_config_data[0], sizeof(sys_config), &bytes_read));
    EXPECT_EQ(FS_NO_ERROR, fs_close(file_system_handle));
    EXPECT_EQ(sizeof(sys_config), bytes_read);

    // Check RAM and FLASH contents match
    bool RAM_FLASH_mismatch = false;
    uint8_t * sys_config_itr = (uint8_t *) &sys_config;
    for (unsigned int i = 0; i < bytes_read; ++i)
    {
        if (flash_config_data[i] != sys_config_itr[i])
        {
            RAM_FLASH_mismatch = true;
            break;
        }
    }

    EXPECT_FALSE(RAM_FLASH_mismatch);
}

TEST_F(SmTest, CfgRestoreNoFile)
{
    startup_provisioning_needed(); // Boot and transition to provisioning needed state

    connect(); // Connect the config_if

    sm_iterate();

    EXPECT_EQ(SM_STATE_PROVISIONING, sm_get_state());

    sm_iterate(); // Queue the first receive

    // Generate cfg save request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_RESTORE_REQ);
    send_message(req, CMD_SIZE_HDR);

    sm_iterate(); // Process the message

    // Check the response
    cmd_t resp;
    resp = receive_message();
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_FILE_NOT_FOUND, resp.p.cmd_generic_resp.error_code);
}

TEST_F(SmTest, CfgRestoreSuccess)
{
    startup_provisioning_needed(); // Boot and transition to provisioning needed state

    connect(); // Connect the config_if

    sm_iterate();

    EXPECT_EQ(SM_STATE_PROVISIONING, sm_get_state());

    sm_iterate(); // Queue the first receive

    // Generate the configuration file in FLASH
    fs_t file_system;
    fs_handle_t file_system_handle;
    uint32_t bytes_written;

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_format(file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_open(file_system, &file_system_handle, FS_FILE_ID_CONF, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(file_system_handle, &sys_config, sizeof(sys_config), &bytes_written));
    EXPECT_EQ(FS_NO_ERROR, fs_close(file_system_handle));

    // Generate cfg save request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_RESTORE_REQ);
    send_message(req, CMD_SIZE_HDR);

    sm_iterate(); // Process the message

    // Check the response
    cmd_t resp;
    resp = receive_message();
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);
}

TEST_F(SmTest, CfgProtectSuccess)
{
    startup_provisioning_needed(); // Boot and transition to provisioning needed state

    connect(); // Connect the config_if

    sm_iterate();

    EXPECT_EQ(SM_STATE_PROVISIONING, sm_get_state());

    sm_iterate(); // Queue the first receive

    // Generate the configuration file in FLASH
    fs_t file_system;
    fs_handle_t file_system_handle;

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_format(file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_open(file_system, &file_system_handle, FS_FILE_ID_CONF, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_close(file_system_handle));

    // Generate cfg save request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_PROTECT_REQ);
    send_message(req, CMD_SIZE_HDR);

    sm_iterate(); // Process the message

    // Check the response
    cmd_t resp;
    resp = receive_message();
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);
}

TEST_F(SmTest, CfgProtectNoFile)
{
    startup_provisioning_needed(); // Boot and transition to provisioning needed state

    connect(); // Connect the config_if

    sm_iterate();

    EXPECT_EQ(SM_STATE_PROVISIONING, sm_get_state());

    sm_iterate(); // Queue the first receive

    // Generate cfg save request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_PROTECT_REQ);
    send_message(req, CMD_SIZE_HDR);

    sm_iterate(); // Process the message

    // Check the response
    cmd_t resp;
    resp = receive_message();
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_FILE_NOT_FOUND, resp.p.cmd_generic_resp.error_code);
}

TEST_F(SmTest, CfgUnprotectSuccess)
{
    startup_provisioning_needed(); // Boot and transition to provisioning needed state

    connect(); // Connect the config_if

    sm_iterate();

    EXPECT_EQ(SM_STATE_PROVISIONING, sm_get_state());

    sm_iterate(); // Queue the first receive

    // Generate the configuration file in FLASH
    fs_t file_system;
    fs_handle_t file_system_handle;

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_format(file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_open(file_system, &file_system_handle, FS_FILE_ID_CONF, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_close(file_system_handle));

    // Generate cfg save request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_UNPROTECT_REQ);
    send_message(req, CMD_SIZE_HDR);

    sm_iterate(); // Process the message

    // Check the response
    cmd_t resp;
    resp = receive_message();
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);
}

TEST_F(SmTest, CfgUnprotectNoFile)
{
    startup_provisioning_needed(); // Boot and transition to provisioning needed state

    connect(); // Connect the config_if

    sm_iterate();

    EXPECT_EQ(SM_STATE_PROVISIONING, sm_get_state());

    sm_iterate(); // Queue the first receive

    // Generate cfg save request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_UNPROTECT_REQ);
    send_message(req, CMD_SIZE_HDR);

    sm_iterate(); // Process the message

    // Check the response
    cmd_t resp;
    resp = receive_message();
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_FILE_NOT_FOUND, resp.p.cmd_generic_resp.error_code);
}