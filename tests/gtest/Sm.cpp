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
#include "Mockconfig_if.h"
#include "Mockfs.h"
#include "sm.h"
#include <stdlib.h>
}

#include "googletest.h"

#include <cstdlib>
#include <cstring>
#include <iostream>

#include <queue>

#define FS_FILE_ID_CONF             (0) // The File ID of the configuration data
#define FS_FILE_ID_STM32_IMAGE      (1) // STM32 application image
#define FS_FILE_ID_BLE_APP_IMAGE    (2) // BLE application image
#define FS_FILE_ID_BLE_SOFT_IMAGE   (3) // BLE soft-device image
#define FS_FILE_ID_LOG              (4) // Sensor log file

// Dummy functions, used to ignore all calls to this function
void syshal_batt_init_dummy(uint32_t instance, int cmock_num_calls) {}
void syshal_gpio_init_dummy(uint32_t pin, int cmock_num_calls) {}
int syshal_uart_init_dummy(uint32_t instance, int cmock_num_calls) {return SYSHAL_UART_NO_ERROR;}
int syshal_spi_init_dummy(uint32_t instance, int cmock_num_calls) {return SYSHAL_SPI_NO_ERROR;}
int syshal_i2c_init_dummy(uint32_t instance, int cmock_num_calls) {return SYSHAL_I2C_NO_ERROR;}
int config_if_init_dummy(config_if_backend_t backend, int cmock_num_calls) {return CONFIG_IF_NO_ERROR;}
int fs_init_dummy(uint32_t device, int cmock_num_calls) {return FS_NO_ERROR;}
int fs_mount_dummy(uint32_t device, fs_t * fs, int cmock_num_calls) {return FS_NO_ERROR;}

/////// File system handlers ///////
bool file_currently_open = false;

// fs_open callback function
std::queue<uint8_t> fs_open_expected_file_id;
std::queue<fs_mode_t> fs_open_expected_mode;
std::queue<int> fs_open_return_value;
int fs_open_callback(fs_t fs, fs_handle_t * handle, uint8_t file_id, fs_mode_t mode, uint8_t * user_flags, int cmock_num_calls)
{
    EXPECT_EQ(fs_open_expected_file_id.front(), file_id);
    fs_open_expected_file_id.pop();
    EXPECT_EQ(fs_open_expected_mode.front(), mode);
    fs_open_expected_mode.pop();

    int ret_val = fs_open_return_value.front();
    fs_open_return_value.pop();

    if (FS_NO_ERROR == ret_val)
        file_currently_open = true;

    return ret_val;
}

// fs_read callback function
int fs_read_return_value;
int fs_read_callback(fs_handle_t handle, void * dest, uint32_t size, uint32_t * read, int cmock_num_calls)
{
    *read = size;
    return fs_read_return_value;
}

// fs_write callback function
int fs_write_return_value;
int fs_write_callback(fs_handle_t handle, const void * src, uint32_t size, uint32_t * written, int cmock_num_calls)
{
    *written = size;
    return fs_write_return_value;
}

// fs_close callback function
int fs_close_callback(fs_handle_t handle, int cmock_num_calls)
{
    file_currently_open = false;
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
        Mockfs_Init();

        // Callbacks
        fs_open_StubWithCallback(fs_open_callback);
        fs_read_StubWithCallback(fs_read_callback);
        fs_write_StubWithCallback(fs_write_callback);
        fs_close_StubWithCallback(fs_close_callback);
    }

    virtual void TearDown()
    {
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
        Mockfs_Verify();
        Mockfs_Destroy();
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
        fs_init_StubWithCallback(fs_init_dummy);
        fs_mount_StubWithCallback(fs_mount_dummy);
    }

    // A call to fs_get_configuration_data where the configuration data file is non-existant
    void fs_get_configuration_data_no_file()
    {
        // fs_open
        fs_open_expected_file_id.push(FS_FILE_ID_CONF);
        fs_open_expected_mode.push(FS_MODE_READONLY);
        fs_open_return_value.push(FS_ERROR_FILE_NOT_FOUND);
    }

    // A call to fs_get_configuration_data where the configuration data is correctly read
    void fs_get_configuration_data_success()
    {
        // fs_open
        fs_open_expected_file_id.push(FS_FILE_ID_CONF);
        fs_open_expected_mode.push(FS_MODE_READONLY);
        fs_open_return_value.push(FS_NO_ERROR);

        // fs_read
        fs_read_return_value = FS_NO_ERROR;
    }

    // A call to fs_create_configuration_data where the configuration data is correctly made
    void fs_create_configuration_data_success()
    {
        // fs_open
        fs_open_expected_file_id.push(FS_FILE_ID_CONF);
        fs_open_expected_mode.push(FS_MODE_CREATE);
        fs_open_return_value.push(FS_NO_ERROR);

        // fs_write
        fs_write_return_value = FS_NO_ERROR;
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

    fs_get_configuration_data_no_file(); // No configuration file present
    fs_create_configuration_data_success(); // Successfully create a file

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

    fs_get_configuration_data_success(); // Successful read of a configuration file

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

    fs_get_configuration_data_success(); // Successful read of a configuration file

    syshal_batt_charging_ExpectAndReturn(true);

    sm_iterate();

    EXPECT_EQ(SM_STATE_STANDBY_BATTERY_CHARGING, sm_get_state());
}

TEST_F(SmTest, BootBatteryLevelLow)
{
    sm_set_state(SM_STATE_BOOT);
    HardwareInit();

    fs_get_configuration_data_success(); // Successful read of a configuration file

    syshal_batt_charging_ExpectAndReturn(false);
    syshal_batt_state_ExpectAndReturn(POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL);

    sm_iterate();

    EXPECT_EQ(SM_STATE_STANDBY_BATTERY_LEVEL_LOW, sm_get_state());
}