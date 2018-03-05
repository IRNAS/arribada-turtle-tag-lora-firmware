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

static uint32_t I2C_handle;

//////////////////////////////////////// Internal functions ////////////////////////////////////////

// Read and return the contents of the flag register
#define ERRORVAL -1

static inline void BQ27621_write(uint8_t regAddress, uint16_t value)
{
    uint8_t data[2];
    data[0] = value >> 8;
    data[1] = value & 0xFF;
    syshal_i2c_write_reg(I2C_handle, BQ27621_ADDR, regAddress, data, 2);
}

static inline uint16_t BQ27621_read(uint8_t regAddress)
{
    uint8_t data[2];
    uint32_t length = syshal_i2c_read_reg(I2C_handle, BQ27621_ADDR, regAddress, data, 2);

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

void syshal_batt_init(uint32_t instance)
{
    I2C_handle = instance;

    BQ27621_set_cfgupdate();
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

syshal_batt_state_t syshal_batt_state(void)
{
    // WARN this would return POWER_SUPPLY_CAPACITY_LEVEL_NORMAL if the BQ27621 was unresponsive

    syshal_batt_state_t level;

    uint16_t flags = BQ27621_read_flags();

    if (flags & BQ27621_FLAG_FC)
        level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
    else if (flags & BQ27621_FLAG_SOC1)
        level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
    else if (flags & BQ27621_FLAG_SOCF)
        level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
    else
        level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;

    return level;
}