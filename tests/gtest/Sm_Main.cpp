// Sm_Main.cpp - Main statemachine unit tests
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

extern "C" {
#include "unity.h"
#include <assert.h>
#include <stdint.h>
//#include "Mocksyshal_axl.h"
#include "Mocksyshal_batt.h"
#include "Mocksyshal_ble.h"
#include "Mocksyshal_gpio.h"
#include "Mocksyshal_gps.h"
#include "Mocksyshal_flash.h"
#include "Mocksyshal_uart.h"
#include "Mocksyshal_spi.h"
#include "Mocksyshal_switch.h"
#include "Mocksyshal_i2c.h"
#include "Mocksyshal_rtc.h"
#include "Mocksyshal_time.h"
#include "Mocksyshal_pmu.h"
#include "Mockconfig_if.h"
#include "fs_priv.h"
#include "fs.h"
//#include "crc32.h"
#include "cmd.h"

#include "sys_config.h"
#include "sm.h"
#include "sm_main.h"
#include <stdlib.h>
}

#include "googletest.h"

#include <cstdlib>
#include <cstring>
#include <iostream>

#include <utility>
#include <queue>
#include <vector>


// config_if
typedef std::vector<uint8_t> message_t;

config_if_backend_t config_if_current_interface;

int config_if_init_GTest(config_if_backend_t backend, int cmock_num_calls)
{
    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current_interface); // We probably shouldn't be calling init when a backend is already in use

    config_if_current_interface = backend;

    return CONFIG_IF_NO_ERROR;
}

int config_if_term_GTest(int cmock_num_calls)
{
    config_if_current_interface = CONFIG_IF_BACKEND_NOT_SET;
    return CONFIG_IF_NO_ERROR;
}

config_if_backend_t config_if_current_GTest(int cmock_num_calls)
{
    return config_if_current_interface;
}

void config_if_tick_GTest(int cmock_num_calls) {}

uint8_t * config_if_receive_buffer;
uint32_t config_if_receive_buffer_size;
bool config_if_receive_queued = false;

static int config_if_receive_GTest(uint8_t * data, uint32_t size, int cmock_num_calls)
{
    if (config_if_receive_queued)
        return CONFIG_IF_ERROR_BUSY;

    config_if_receive_queued = true;

    config_if_receive_buffer = data;
    config_if_receive_buffer_size = size;

    return CONFIG_IF_NO_ERROR;
}

std::vector< std::vector<uint8_t> > config_if_transmitted_data;

static int config_if_send_GTest(uint8_t * data, uint32_t size, int cmock_num_calls)
{
    config_if_transmitted_data.push_back( std::vector<uint8_t> (data, data + size) );

    return CONFIG_IF_NO_ERROR;
}

// Send config_if message to the state machine
void send_message(uint8_t * message, uint32_t size)
{
    // We should have a configuration interface init before sending a message
    ASSERT_NE(CONFIG_IF_BACKEND_NOT_SET, config_if_current_interface);

    ASSERT_LE(size, config_if_receive_buffer_size); // Message is less than or equal to expected size
    ASSERT_NE(nullptr, config_if_receive_buffer);

    // Copy message into receive buffer
    memcpy(config_if_receive_buffer, message, size);

    // Generate a receive complete event
    config_if_event_t event;
    event.id = CONFIG_IF_EVENT_RECEIVE_COMPLETE;
    event.backend = config_if_current_interface;
    event.receive.size = size;

    config_if_receive_queued = false;
    config_if_callback(&event);
}

// Send config_if message to the state machine
void send_message(cmd_t * message, uint32_t size)
{
    send_message((uint8_t *)message, size);
}

void receive_message(cmd_t * message)
{
    ASSERT_GT(config_if_transmitted_data.size(), 0); // We must at least have a message to receive

    // Prepare a transmit complete event
    config_if_event_t event;
    event.id = CONFIG_IF_EVENT_SEND_COMPLETE;
    event.backend = config_if_current_interface;
    event.send.size = config_if_transmitted_data.size();

    // Copy message from send buffer
    for (auto i = 0; i < config_if_transmitted_data[0].size(); ++i)
        ((uint8_t *)message)[i] = config_if_transmitted_data.back()[i];

    // Remove this buffer from the vector
    config_if_transmitted_data.pop_back();

    config_if_callback(&event); // Generate the transmit complete event
}

// syshal_time

int syshal_time_init_GTest(int cmock_num_calls) {return SYSHAL_TIME_NO_ERROR;}

uint32_t syshal_time_get_ticks_ms_value;
uint32_t syshal_time_get_ticks_ms_GTest(int cmock_num_calls)
{
    return syshal_time_get_ticks_ms_value;
}

// syshal_gpio
bool GPIO_pin_input_state[GPIO_TOTAL_NUMBER];
bool GPIO_pin_output_state[GPIO_TOTAL_NUMBER];
void (*GPIO_callback_function[GPIO_TOTAL_NUMBER])(void);

int syshal_gpio_init_GTest(uint32_t pin, int cmock_num_calls) {return SYSHAL_GPIO_NO_ERROR;}
bool syshal_gpio_get_input_GTest(uint32_t pin, int cmock_num_calls) {return GPIO_pin_input_state[pin];}
void syshal_gpio_set_output_low_GTest(uint32_t pin, int cmock_num_calls) {GPIO_pin_output_state[pin] = 0;}
void syshal_gpio_set_output_high_GTest(uint32_t pin, int cmock_num_calls) {GPIO_pin_output_state[pin] = 1;}
void syshal_gpio_set_output_toggle_GTest(uint32_t pin, int cmock_num_calls) {GPIO_pin_output_state[pin] = !GPIO_pin_output_state[pin];}
void syshal_gpio_enable_interrupt_GTest(uint32_t pin, void (*callback_function)(void), int cmock_num_calls) {GPIO_callback_function[pin] = callback_function;}

// syshal_ble
const uint32_t syshal_ble_version = 0xABCD0123;

int syshal_ble_get_version_GTest(uint32_t * version, int cmock_num_calls)
{
    *version = syshal_ble_version;
    return SYSHAL_BLE_NO_ERROR;
};

// syshal_rtc
syshal_rtc_data_and_time_t current_date_time;

int syshal_rtc_init_GTest(int cmock_num_calls) {return SYSHAL_RTC_NO_ERROR;}

int syshal_rtc_set_date_and_time_GTest(syshal_rtc_data_and_time_t date_time, int cmock_num_calls)
{
    current_date_time = date_time;
    return SYSHAL_RTC_NO_ERROR;
}

int syshal_rtc_get_date_and_time_GTest(syshal_rtc_data_and_time_t * date_time, int cmock_num_calls)
{
    *date_time = current_date_time;
    return SYSHAL_RTC_NO_ERROR;
}

// syshal_batt
int battery_level;

int syshal_batt_level_GTest(int cmock_num_calls)
{
    return battery_level;
}

// syshal_uart
int syshal_uart_init_GTest(uint32_t instance, int cmock_num_calls) {return SYSHAL_UART_NO_ERROR;}

// syshal_spi
int syshal_spi_init_GTest(uint32_t instance, int cmock_num_calls) {return SYSHAL_SPI_NO_ERROR;}

// syshal_i2c
int syshal_i2c_init_GTest(uint32_t instance, int cmock_num_calls) {return SYSHAL_I2C_NO_ERROR;}

// syshal_flash
#define FLASH_SIZE          (FS_PRIV_SECTOR_SIZE * FS_PRIV_MAX_SECTORS)
#define ASCII(x)            ((x) >= 32 && (x) <= 127) ? (x) : '.'

static bool fs_trace;
char flash_ram[FLASH_SIZE];

int syshal_flash_init_GTest(uint32_t drive, uint32_t device, int cmock_num_calls) {return SYSHAL_FLASH_NO_ERROR;}

int syshal_flash_read_GTest(uint32_t device, void * dest, uint32_t address, uint32_t size, int cmock_num_calls)
{
    //printf("syshal_flash_read(%08x,%u)\n", address, size);
    for (unsigned int i = 0; i < size; i++)
        ((char *)dest)[i] = flash_ram[address + i];

    return 0;
}

int syshal_flash_write_GTest(uint32_t device, const void * src, uint32_t address, uint32_t size, int cmock_num_calls)
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

int syshal_flash_erase_GTest(uint32_t device, uint32_t address, uint32_t size, int cmock_num_calls)
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

// syshal_gps
void syshal_gps_init_GTest(int cmock_num_calls) {}
void syshal_gps_shutdown_GTest(int cmock_num_calls) {}

// syshal_switch
bool syshal_switch_state;

int syshal_switch_init_GTest(int cmock_num_calls) {return SYSHAL_SWITCH_NO_ERROR;}
bool syshal_switch_get_GTest(int cmock_num_calls) {return syshal_switch_state;}

class Sm_MainTest : public ::testing::Test
{

    virtual void SetUp()
    {
        // config_if
        Mockconfig_if_Init();

        config_if_init_StubWithCallback(config_if_init_GTest);
        config_if_term_StubWithCallback(config_if_term_GTest);
        config_if_current_StubWithCallback(config_if_current_GTest);
        config_if_send_StubWithCallback(config_if_send_GTest);
        config_if_receive_StubWithCallback(config_if_receive_GTest);
        config_if_tick_StubWithCallback(config_if_tick_GTest);

        config_if_current_interface = CONFIG_IF_BACKEND_NOT_SET;

        // syshal_gpio
        Mocksyshal_gpio_Init();

        syshal_gpio_init_StubWithCallback(syshal_gpio_init_GTest);
        syshal_gpio_get_input_StubWithCallback(syshal_gpio_get_input_GTest);
        syshal_gpio_set_output_low_StubWithCallback(syshal_gpio_set_output_low_GTest);
        syshal_gpio_set_output_high_StubWithCallback(syshal_gpio_set_output_high_GTest);
        syshal_gpio_set_output_toggle_StubWithCallback(syshal_gpio_set_output_toggle_GTest);
        syshal_gpio_enable_interrupt_StubWithCallback(syshal_gpio_enable_interrupt_GTest);

        // Set all gpio input pins low
        for (unsigned int i = 0; i < GPIO_TOTAL_NUMBER; ++i)
            GPIO_pin_input_state[i] = 0;

        // Set all gpio output pins low
        for (unsigned int i = 0; i < GPIO_TOTAL_NUMBER; ++i)
            GPIO_pin_output_state[i] = 0;

        // Clear all gpio interrupts
        for (unsigned int i = 0; i < GPIO_TOTAL_NUMBER; ++i)
            GPIO_callback_function[i] = nullptr;

        // syshal_time
        Mocksyshal_time_Init();

        syshal_time_init_StubWithCallback(syshal_time_init_GTest);
        syshal_time_get_ticks_ms_StubWithCallback(syshal_time_get_ticks_ms_GTest);
        syshal_time_get_ticks_ms_value = 0;

        // syshal_ble
        Mocksyshal_ble_Init();
        syshal_ble_get_version_StubWithCallback(syshal_ble_get_version_GTest);

        // syshal_rtc
        Mocksyshal_rtc_Init();

        syshal_rtc_init_StubWithCallback(syshal_rtc_init_GTest);
        syshal_rtc_set_date_and_time_StubWithCallback(syshal_rtc_set_date_and_time_GTest);
        syshal_rtc_get_date_and_time_StubWithCallback(syshal_rtc_get_date_and_time_GTest);

        // syshal_batt
        Mocksyshal_batt_Init();

        syshal_batt_level_StubWithCallback(syshal_batt_level_GTest);

        battery_level = 100;

        // sys_config

        // Unset all tags
        uint16_t last_index = 0;
        uint16_t tag;

        while (!sys_config_iterate(&tag, &last_index))
        {
            sys_config_unset(tag);
        }

        // syshal_uart
        Mocksyshal_uart_Init();
        syshal_uart_init_StubWithCallback(syshal_uart_init_GTest);

        // syshal_spi
        Mocksyshal_spi_Init();
        syshal_spi_init_StubWithCallback(syshal_spi_init_GTest);

        // syshal_i2c
        Mocksyshal_i2c_Init();
        syshal_i2c_init_StubWithCallback(syshal_i2c_init_GTest);

        // syshal_flash
        Mocksyshal_flash_Init();
        syshal_flash_init_StubWithCallback(syshal_flash_init_GTest);
        syshal_flash_read_StubWithCallback(syshal_flash_read_GTest);
        syshal_flash_write_StubWithCallback(syshal_flash_write_GTest);
        syshal_flash_erase_StubWithCallback(syshal_flash_erase_GTest);

        // Clear FLASH contents
        for (auto i = 0; i < FLASH_SIZE; ++i)
            flash_ram[i] = 0xFF;

        fs_trace = false; // turn FS trace off

        // syshal_gps
        Mocksyshal_gps_Init();
        syshal_gps_init_StubWithCallback(syshal_gps_init_GTest);
        syshal_gps_shutdown_StubWithCallback(syshal_gps_shutdown_GTest);

        // syshal_switch
        Mocksyshal_switch_Init();
        syshal_switch_init_StubWithCallback(syshal_switch_init_GTest);
        syshal_switch_get_StubWithCallback(syshal_switch_get_GTest);

        // Setup main state machine
        sm_init(&state_handle, sm_main_states);

        sys_config.format_version = SYS_CONFIG_FORMAT_VERSION;
    }

    virtual void TearDown()
    {
        Mockconfig_if_Verify();
        Mockconfig_if_Destroy();
        Mocksyshal_gpio_Verify();
        Mocksyshal_gpio_Destroy();
        Mocksyshal_time_Verify();
        Mocksyshal_time_Destroy();
        Mocksyshal_ble_Verify();
        Mocksyshal_ble_Destroy();
        Mocksyshal_rtc_Verify();
        Mocksyshal_rtc_Destroy();
        Mocksyshal_batt_Verify();
        Mocksyshal_batt_Destroy();
        Mocksyshal_uart_Verify();
        Mocksyshal_uart_Destroy();
        Mocksyshal_spi_Verify();
        Mocksyshal_spi_Destroy();
        Mocksyshal_i2c_Verify();
        Mocksyshal_i2c_Destroy();
        Mocksyshal_flash_Verify();
        Mocksyshal_flash_Destroy();
        Mocksyshal_gps_Verify();
        Mocksyshal_gps_Destroy();
    }

public:

    sm_handle_t state_handle;

    void BootTagsNotSet(void)
    {
        sm_set_current_state(&state_handle, SM_MAIN_BOOT);

        sm_tick(&state_handle);
    }

    void SetVUSB(bool state)
    {
        GPIO_pin_input_state[GPIO_VUSB] = state;
    }

    void SetBatteryPercentage(int level)
    {
        battery_level = level;
    }

    void SetBatteryLowThreshold(uint8_t level)
    {
        sys_config.sys_config_battery_low_threshold.hdr.set = true;
        sys_config.sys_config_battery_low_threshold.contents.threshold = level;
    }

    // Message handling //

    static void BLEConnectionEvent(void)
    {
        config_if_event_t event;
        event.id = CONFIG_IF_EVENT_CONNECTED;
        event.backend = CONFIG_IF_BACKEND_BLE;
        config_if_callback(&event);
    }

    static void BLEDisconnectEvent(void)
    {
        config_if_event_t event;
        event.id = CONFIG_IF_EVENT_DISCONNECTED;
        event.backend = CONFIG_IF_BACKEND_BLE;
        config_if_callback(&event);
    }

    static void USBConnectionEvent(void)
    {
        config_if_event_t event;
        event.id = CONFIG_IF_EVENT_CONNECTED;
        event.backend = CONFIG_IF_BACKEND_USB;
        config_if_callback(&event);
    }

    static void USBDisconnectEvent(void)
    {
        config_if_event_t event;
        event.id = CONFIG_IF_EVENT_DISCONNECTED;
        event.backend = CONFIG_IF_BACKEND_USB;
        config_if_callback(&event);
    }

    static void set_all_configuration_tags_RAM()
    {
        uint8_t config_if_dummy_data[SYS_CONFIG_MAX_DATA_SIZE];
        for (auto i = 0; i < SYS_CONFIG_MAX_DATA_SIZE; ++i)
            config_if_dummy_data[i] = rand();

        uint32_t length = 0;

        uint16_t tag, last_index = 0;
        while (!sys_config_iterate(&tag, &last_index))
        {
            void * src;
            int ret;
            do
            {
                int ret = sys_config_set(tag, &src, length++);
            }
            while (SYS_CONFIG_ERROR_WRONG_SIZE == ret &&
                   length < SYS_CONFIG_MAX_DATA_SIZE);

            length = 0;

            if (SYS_CONFIG_ERROR_NO_MORE_TAGS != ret)
            {
                break;
            }
        }
    }

};

//////////////////////////////////////////////////////////////////
/////////////////////////// Boot State ///////////////////////////
//////////////////////////////////////////////////////////////////

TEST_F(Sm_MainTest, BootNoTags)
{
    BootTagsNotSet();

    EXPECT_EQ(SM_MAIN_PROVISIONING_NEEDED, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, BootNoTagsVUSB)
{
    SetVUSB(true);

    BootTagsNotSet();

    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));
}

//////////////////////////////////////////////////////////////////
/////////////////////// Battery Low State ////////////////////////
//////////////////////////////////////////////////////////////////

TEST_F(Sm_MainTest, BatteryLowToBatteryChargingNoVUSB)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_BATTERY_LEVEL_LOW);

    SetVUSB(false);

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_BATTERY_LEVEL_LOW, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, BatteryLowToBatteryChargingVUSB)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_BATTERY_LEVEL_LOW);

    SetVUSB(true);

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));
}

//////////////////////////////////////////////////////////////////
///////////////////// Battery Charging State /////////////////////
//////////////////////////////////////////////////////////////////

TEST_F(Sm_MainTest, BatteryChargingNoVUSBBatteryLowNoThreshold)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_BATTERY_CHARGING);

    SetVUSB(false);
    SetBatteryPercentage(0);

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING_NEEDED, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, BatteryChargingNoVUSBBatteryLow)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_BATTERY_CHARGING);

    SetVUSB(false);
    SetBatteryPercentage(0);

    SetBatteryLowThreshold(10);

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_BATTERY_LEVEL_LOW, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, BatteryChargingUSBTimeout)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_BATTERY_CHARGING);

    SetVUSB(true);

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());
    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));

    // Jump forward 5 seconds
    syshal_time_get_ticks_ms_value = 5 * 1000;

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());
    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));

    // Jump forward 100 seconds
    syshal_time_get_ticks_ms_value = 100 * 1000;

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());
    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, BatteryChargingBLERunningButNotConnected)
{
    // This should realise the BLE stack is running but is not connected
    // So it should terminate it and start the USB stack
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_BATTERY_CHARGING);

    config_if_init(CONFIG_IF_BACKEND_BLE);
    SetVUSB(true);

    EXPECT_EQ(CONFIG_IF_BACKEND_BLE, config_if_current());

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());
    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, BatteryChargingUSBConnected)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_BATTERY_CHARGING);

    SetVUSB(true);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());
    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));

    USBConnectionEvent();

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
}

//////////////////////////////////////////////////////////////////
////////////////////// Log File Full State ///////////////////////
//////////////////////////////////////////////////////////////////

TEST_F(Sm_MainTest, LogFileFullToBatteryCharging)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_LOG_FILE_FULL);

    EXPECT_EQ(SM_MAIN_LOG_FILE_FULL, sm_get_current_state(&state_handle));

    SetVUSB(true);

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, LogFileFullToLowBattery)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_LOG_FILE_FULL);

    EXPECT_EQ(SM_MAIN_LOG_FILE_FULL, sm_get_current_state(&state_handle));

    SetBatteryPercentage(0);
    SetBatteryLowThreshold(10);

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_BATTERY_LEVEL_LOW, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, LogFileFullBLEConnection)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_LOG_FILE_FULL);

    EXPECT_EQ(SM_MAIN_LOG_FILE_FULL, sm_get_current_state(&state_handle));

    config_if_init(CONFIG_IF_BACKEND_BLE);
    BLEConnectionEvent();

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
}

//////////////////////////////////////////////////////////////////
/////////////////////// Provisioning State ///////////////////////
//////////////////////////////////////////////////////////////////

TEST_F(Sm_MainTest, ProvisioningToProvisioningNeeded)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_BLE);
    BLEConnectionEvent();

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));

    BLEDisconnectEvent();

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING_NEEDED, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, ProvisioningToCharging)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    SetVUSB(true);
    USBConnectionEvent();

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));

    USBDisconnectEvent();

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, ProvisioningToLowBattery)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();
    SetBatteryPercentage(0);
    SetBatteryLowThreshold(10);

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_BATTERY_LEVEL_LOW, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());
}

//////////////////////////////////////////////////////////////////
//////////////////////// Message Handling ////////////////////////
//////////////////////////////////////////////////////////////////

TEST_F(Sm_MainTest, StatusRequest)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate status request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_STATUS_REQ);
    send_message(&req, CMD_SIZE_HDR);

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_STATUS_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_status_resp.error_code);
    EXPECT_EQ(STM32_FIRMWARE_VERSION, resp.p.cmd_status_resp.stm_firmware_version);
    EXPECT_EQ(syshal_ble_version, resp.p.cmd_status_resp.ble_firmware_version);
    EXPECT_EQ(SYS_CONFIG_FORMAT_VERSION, resp.p.cmd_status_resp.configuration_format_version);
}

TEST_F(Sm_MainTest, ResetRequest)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate status request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_RESET_REQ);
    req.p.cmd_reset_req.reset_type = 0;
    send_message(&req, CMD_SIZE(cmd_reset_req_t));

    syshal_pmu_reset_Expect();

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);
}

TEST_F(Sm_MainTest, CfgWriteOne)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate cfg write request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_WRITE_REQ);
    req.p.cmd_cfg_write_req.length = SYS_CONFIG_TAG_ID_SIZE + SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_group_sensor_readings_enable_t);
    send_message(&req, CMD_SIZE(cmd_cfg_write_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);

    // Generate cfg tag data packet
    uint8_t tag_data_packet[3];
    tag_data_packet[0] = uint16_t(SYS_CONFIG_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE) & 0x00FF;
    tag_data_packet[1] = (uint16_t(SYS_CONFIG_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE) & 0xFF00) >> 8;
    tag_data_packet[2] = true; // Enable

    send_message(tag_data_packet, sizeof(tag_data_packet));

    sm_tick(&state_handle); // Process the message

    // Check the response
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_CFG_WRITE_CNF, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_cfg_write_cnf.error_code);

    // Check the tag was correctly set
    EXPECT_TRUE(sys_config.sys_config_logging_group_sensor_readings_enable.contents.enable);
}

TEST_F(Sm_MainTest, CfgWriteOneInvalidTag)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate cfg write request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_WRITE_REQ);
    req.p.cmd_cfg_write_req.length = SYS_CONFIG_TAG_ID_SIZE + SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_group_sensor_readings_enable_t);
    send_message(&req, CMD_SIZE(cmd_cfg_write_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);

    // Generate cfg tag data packet
    uint8_t tag_data_packet[3];
    uint16_t invalid_tag_ID = 0xABAB;
    tag_data_packet[0] = invalid_tag_ID & 0x00FF;
    tag_data_packet[1] = (invalid_tag_ID & 0xFF00) >> 8;
    tag_data_packet[2] = true; // Enable

    send_message(tag_data_packet, sizeof(tag_data_packet));

    sm_tick(&state_handle); // Process the message
    sm_tick(&state_handle); // Return the error

    // Check the response
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_CFG_WRITE_CNF, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_INVALID_CONFIG_TAG, resp.p.cmd_cfg_write_cnf.error_code);
}

TEST_F(Sm_MainTest, CfgReadOne)
{
    // Set a tag up for reading later
    sys_config.sys_config_logging_group_sensor_readings_enable.contents.enable = true;
    sys_config.sys_config_logging_group_sensor_readings_enable.hdr.set = true;

    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate cfg read request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_READ_REQ);
    req.p.cmd_cfg_read_req.configuration_tag = SYS_CONFIG_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE;
    send_message(&req, CMD_SIZE(cmd_cfg_read_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_CFG_READ_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_cfg_read_resp.error_code);
    uint32_t expected_length = resp.p.cmd_cfg_read_resp.length;

    sm_tick(&state_handle); // Send the data packet

    receive_message(&resp);
    uint8_t * message = (uint8_t *) &resp;

    uint16_t tag = 0;
    tag |= (uint16_t) message[0] & 0x00FF;
    tag |= (uint16_t) (message[1] << 8) & 0xFF00;
    EXPECT_EQ(SYS_CONFIG_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE, tag);

    // Check the tag was correctly read
    EXPECT_TRUE(message[2]);
}

TEST_F(Sm_MainTest, CfgSaveSuccess)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    // Generate cfg save request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_SAVE_REQ);
    send_message((uint8_t *) &req, CMD_SIZE_HDR);

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
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

TEST_F(Sm_MainTest, CfgRestoreNoFile)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    // Generate cfg save request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_RESTORE_REQ);
    send_message((uint8_t *) &req, CMD_SIZE_HDR);

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_FILE_NOT_FOUND, resp.p.cmd_generic_resp.error_code);
}

TEST_F(Sm_MainTest, CfgRestoreSuccess)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

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
    send_message((uint8_t *) &req, CMD_SIZE_HDR);

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);
}

TEST_F(Sm_MainTest, CfgProtectSuccess)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

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
    send_message((uint8_t *) &req, CMD_SIZE_HDR);

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);
}

TEST_F(Sm_MainTest, CfgProtectNoFile)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    // Generate cfg save request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_PROTECT_REQ);
    send_message((uint8_t *) &req, CMD_SIZE_HDR);

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_FILE_NOT_FOUND, resp.p.cmd_generic_resp.error_code);
}

TEST_F(Sm_MainTest, CfgUnprotectSuccess)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

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
    send_message((uint8_t *) &req, CMD_SIZE_HDR);

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);
}

TEST_F(Sm_MainTest, CfgUnprotectNoFile)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    // Generate cfg save request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_UNPROTECT_REQ);
    send_message((uint8_t *) &req, CMD_SIZE_HDR);

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_FILE_NOT_FOUND, resp.p.cmd_generic_resp.error_code);
}

TEST_F(Sm_MainTest, CfgEraseAll)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    set_all_configuration_tags_RAM(); // Set all the configuration tags

    // Generate cfg erase request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_ERASE_REQ);
    req.p.cmd_cfg_erase_req.configuration_tag = CFG_ERASE_REQ_ERASE_ALL;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_cfg_erase_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);

    // Check all the configuration tags have been unset
    bool all_tags_unset = true;
    uint16_t tag, last_index = 0;
    while (!sys_config_iterate(&tag, &last_index))
    {
        void * src;
        int ret = sys_config_get(tag, &src);

        if (SYS_CONFIG_ERROR_TAG_NOT_SET != ret &&
            SYS_CONFIG_TAG_RTC_CURRENT_DATE_AND_TIME != tag &&
            SYS_CONFIG_TAG_LOGGING_FILE_SIZE != tag &&
            SYS_CONFIG_TAG_LOGGING_FILE_TYPE != tag &&
            SYS_CONFIG_TAG_LOGGING_START_END_SYNC_ENABLE != tag)
        {
            all_tags_unset = false;
            break;
        }
    }

    EXPECT_TRUE(all_tags_unset);
}

TEST_F(Sm_MainTest, CfgEraseOne)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    set_all_configuration_tags_RAM(); // Set all the configuration tags

    // Generate log erase request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_ERASE_REQ);
    req.p.cmd_cfg_erase_req.configuration_tag = SYS_CONFIG_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_cfg_erase_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);

    // Check the configuration tag has been unset
    void * src;
    int ret = sys_config_get(SYS_CONFIG_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE, &src);

    bool tag_unset = false;
    if (SYS_CONFIG_ERROR_TAG_NOT_SET == ret)
        tag_unset = true;

    EXPECT_TRUE(tag_unset);
}

TEST_F(Sm_MainTest, LogEraseSuccess)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    // Generate the log file in FLASH
    fs_t file_system;
    fs_handle_t file_system_handle;

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_format(file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_open(file_system, &file_system_handle, FS_FILE_ID_LOG, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_close(file_system_handle));

    // Generate log erase request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_LOG_ERASE_REQ);
    send_message((uint8_t *) &req, CMD_SIZE_HDR);

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);
}

TEST_F(Sm_MainTest, LogEraseNoFile)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    // Generate log erase request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_LOG_ERASE_REQ);
    send_message((uint8_t *) &req, CMD_SIZE_HDR);

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_FILE_NOT_FOUND, resp.p.cmd_generic_resp.error_code);
}

TEST_F(Sm_MainTest, LogCreateFill)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    // Generate log create request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_LOG_CREATE_REQ);
    req.p.cmd_log_create_req.mode = CMD_LOG_CREATE_REQ_MODE_FILL;
    req.p.cmd_log_create_req.sync_enable = false;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_log_create_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);

    // Check the log file has been created as is of the right mode
    fs_t file_system;
    fs_handle_t file_system_handle;
    fs_stat_t file_stats;

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_stat(file_system, FS_FILE_ID_LOG, &file_stats));
    EXPECT_FALSE(file_stats.is_circular);
}

TEST_F(Sm_MainTest, LogCreateCircular)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    // Generate log create request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_LOG_CREATE_REQ);
    req.p.cmd_log_create_req.mode = CMD_LOG_CREATE_REQ_MODE_CIRCULAR;
    req.p.cmd_log_create_req.sync_enable = false;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_log_create_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);

    // Check the log file has been created as is of the right mode
    fs_t file_system;
    fs_handle_t file_system_handle;
    fs_stat_t file_stats;

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_stat(file_system, FS_FILE_ID_LOG, &file_stats));
    EXPECT_TRUE(file_stats.is_circular);
}

TEST_F(Sm_MainTest, LogCreateAlreadyExists)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    // Create the log file
    fs_t file_system;
    fs_handle_t file_system_handle;

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_format(file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_open(file_system, &file_system_handle, FS_FILE_ID_LOG, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_close(file_system_handle));

    // Generate log create request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_LOG_CREATE_REQ);
    req.p.cmd_log_create_req.mode = CMD_LOG_CREATE_REQ_MODE_CIRCULAR;
    req.p.cmd_log_create_req.sync_enable = false;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_log_create_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_FILE_ALREADY_EXISTS, resp.p.cmd_generic_resp.error_code);
}