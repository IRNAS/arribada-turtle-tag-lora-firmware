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
#include "Mocksyshal_gpio.h"
#include "Mocksyshal_gps.h"
#include "Mocksyshal_flash.h"
#include "Mocksyshal_uart.h"
#include "Mocksyshal_spi.h"
#include "Mocksyshal_switch.h"
#include "Mocksyshal_i2c.h"
#include "Mocksyshal_rtc.h"
#include "Mocksyshal_time.h"
#include "Mockconfig_if.h"
#include "fs_priv.h"
#include "fs.h"
//#include "crc32.h"
//#include "cmd.h"

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

int config_if_send_GTest(uint8_t * data, uint32_t size, int cmock_num_calls) {return CONFIG_IF_NO_ERROR;}
int config_if_receive_GTest(uint8_t * data, uint32_t size, int cmock_num_calls) {return CONFIG_IF_NO_ERROR;}

void config_if_tick_GTest(int cmock_num_calls) {}

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
        Mocksyshal_gpio_Init()

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
    }

    virtual void TearDown()
    {
        Mockconfig_if_Verify();
        Mockconfig_if_Destroy();
        Mocksyshal_gpio_Verify();
        Mocksyshal_gpio_Destroy();
        Mocksyshal_time_Verify();
        Mocksyshal_time_Destroy();
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

    void BLEConnectionEvent(void)
    {
        config_if_event_t event;
        event.id = CONFIG_IF_EVENT_CONNECTED;
        event.backend = CONFIG_IF_BACKEND_BLE;
        config_if_callback(&event);
    }

    void BLEDisconnectEvent(void)
    {
        config_if_event_t event;
        event.id = CONFIG_IF_EVENT_DISCONNECTED;
        event.backend = CONFIG_IF_BACKEND_BLE;
        config_if_callback(&event);
    }

    void USBConnectionEvent(void)
    {
        config_if_event_t event;
        event.id = CONFIG_IF_EVENT_CONNECTED;
        event.backend = CONFIG_IF_BACKEND_USB;
        config_if_callback(&event);
    }

    void USBDisconnectEvent(void)
    {
        config_if_event_t event;
        event.id = CONFIG_IF_EVENT_DISCONNECTED;
        event.backend = CONFIG_IF_BACKEND_USB;
        config_if_callback(&event);
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

    sys_config.sys_config_battery_low_threshold.hdr.set = true;
    sys_config.sys_config_battery_low_threshold.contents.threshold = 10;

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