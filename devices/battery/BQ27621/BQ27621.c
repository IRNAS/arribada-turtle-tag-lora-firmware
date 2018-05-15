/* BQ27621.c - HAL for battery monitoring using the BQ27621-G1 fuel gauge
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

// Datasheet: http://www.ti.com/product/bq27621-g1

#include <stdbool.h>
#include "BQ27621.h"
#include "syshal_i2c.h"
#include "syshal_batt.h"
#include "syshal_time.h"
#include "debug.h"
#include "bsp.h"

//////////////////////////////////////// Internal functions ////////////////////////////////////////

// Read and return the contents of the flag register
#define ERRORVAL -1

static inline void BQ27621_write(uint8_t regAddress, uint16_t value)
{
    uint8_t data[2];
    data[0] = value >> 8;
    data[1] = value & 0xFF;
    syshal_i2c_write_reg(I2C_BATTERY, BQ27621_ADDR, regAddress, data, 2);
}

static inline uint16_t BQ27621_read(uint8_t regAddress)
{
    uint8_t data[2];
    uint32_t length = syshal_i2c_read_reg(I2C_BATTERY, BQ27621_ADDR, regAddress, data, 2);

    if (2 == length)
        return (data[0] << 8) | (data[1] & 0xff);
    else
        return 0;
}

static inline uint16_t BQ27621_read_flags(void)
{
    return BQ27621_read(BQ27621_REG_FLAGS);
}

static int BQ27621_cfgupdate(bool active)
{
    const uint32_t limit = 100;
    uint32_t try = limit;
    uint16_t status;
    uint16_t cmd = active ? BQ27621_SET_CFGUPDATE : BQ27621_SOFT_RESET;

    BQ27621_write(BQ27621_REG_CTRL, cmd);

    do
    {
        syshal_time_delay_ms(25);
        status = BQ27621_read_flags();
    }
    while (!!(status & BQ27621_FLAG_CFGUP) != active && --try);

    if (!try)
    {
        DEBUG_PR_ERROR("Timed out waiting for cfgupdate flag %d\n", active);
        return ERRORVAL;
    }

    if (limit - try > 3)
            DEBUG_PR_WARN("Cfgupdate %d, retries %lu\n", active, limit - try);

    return 0;
}

static inline int BQ27621_set_cfgupdate(void)
{
    int status = BQ27621_cfgupdate(true);
    if (ERRORVAL == status)
        DEBUG_PR_ERROR("Bus error on set_cfgupdate: %d\n", status);

    return status;
}

static inline int BQ27621_soft_reset(void)
{
    int status = BQ27621_cfgupdate(false);
    if (ERRORVAL == status)
        DEBUG_PR_ERROR("Bus error on soft_reset: %d\n", status);

    return status;
}

//////////////////////////////////////// Exposed functions ////////////////////////////////////////

int syshal_batt_init(void)
{
    //BQ27621_set_cfgupdate();
    if (syshal_i2c_is_device_ready(I2C_BATTERY, BQ27621_ADDR) != SYSHAL_I2C_NO_ERROR)
    {
        DEBUG_PR_ERROR("BQ27621 unresponsive");
        return SYSHAL_BATT_ERROR_DEVICE_UNRESPONSIVE;
    }

    // FIXME: Wait for initialised flag to be set

    return SYSHAL_BATT_NO_ERROR;
}

// Return the battery temperature in tenths of degree Kelvin
uint16_t syshal_batt_temp(void)
{
    return BQ27621_read(BQ27621_REG_TEMP);
}

// Return the battery Voltage in millivolts
uint16_t syshal_batt_voltage(void)
{
    return BQ27621_read(BQ27621_REG_VOLT);
}

/**
 * @brief      Returns the percentage charge left in the battery
 *
 * @return     0 -> 100%
 */
int syshal_batt_level(void)
{
    uint8_t level;

    int status = syshal_i2c_read_reg(I2C_BATTERY, BQ27621_ADDR, BQ27621_REG_STATE_OF_CHARGE, &level, 1);

    if (SYSHAL_I2C_ERROR_TIMEOUT == status)
        return SYSHAL_BATT_ERROR_TIMEOUT;
    else if (status < 0)
        return SYSHAL_BATT_ERROR_DEVICE_UNRESPONSIVE;
    else
        return level;
}

/*
    uint8_t level;

    int status = syshal_i2c_read_reg(I2C_BATTERY, BQ27621_ADDR, BQ27621_REG_STATE_OF_CHARGE, &level, 1);

    if (SYSHAL_BATT_NO_ERROR == i2c_error_map[status])
        return level;
    else
        return i2c_error_map[status];
*/

/**
 * @brief      Is the battery currently in a charging state?
 *
 * @return     True if charging, false if not
 */
bool syshal_batt_charging(void)
{
    // Read the BQ27621_REG_FLAGS register
//    uint8_t data[2];
//
//    syshal_i2c_read_reg(I2C_BATTERY, BQ27621_ADDR, BQ27621_REG_FLAGS, data, 2);
//    uint16_t flags = (uint16_t) (data[0] << 8) | (data[1] & 0xff);
//
//    // Check for errors
//    if (SYSHAL_I2C_ERROR_TIMEOUT == status)
//        return SYSHAL_BATT_ERROR_TIMEOUT;
//    else if (status < 0)
//        return SYSHAL_BATT_ERROR_DEVICE_UNRESPONSIVE;
//    // Read the discharging flag to determine if we're currently charging
//    return !(flags && BQ27621_FLAG_DSG);

    return false;
}