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
#include "syshal_gpio.h"
#include "debug.h"
#include "bsp.h"

//////////////////////////////////////// Internal functions ////////////////////////////////////////

// Read and return the contents of the flag register
#define ERRORVAL -1
#define BQ27621_TIME_BETWEEN_TRANSFERS_MS (1)
#define BQ27621_NUM_OF_ZERO_CHARGE_REQUIRED (10)

static uint32_t number_of_charge_levels_zero;

static inline int BQ27621_write(uint8_t regAddress, uint16_t value)
{
    uint8_t data[2];
    data[1] = (uint8_t) (value >> 8);
    data[0] = (uint8_t) (value & 0xFF);
    return syshal_i2c_write_reg(I2C_BATTERY, BQ27621_ADDR, regAddress, data, 2);
}

static inline int BQ27621_read(uint8_t regAddress, uint16_t * value)
{
    uint8_t read_bytes[2];
    int length = syshal_i2c_read_reg(I2C_BATTERY, BQ27621_ADDR, regAddress, read_bytes, sizeof(read_bytes));

    if (2 != length)
        return length;

    *value = (uint16_t) ( ((uint16_t)(read_bytes[1]) << 8) | ((uint16_t)(read_bytes[0]) & 0xff) );

    return 0;
}

static inline int BQ27621_read_flags(uint16_t * flags)
{
    return BQ27621_read(BQ27621_REG_FLAGS, flags);
}

static int BQ27621_cfgupdate(bool active)
{
    const uint32_t limit = 100;
    uint32_t try = limit;
    uint16_t status = 0;
    uint16_t cmd = active ? BQ27621_SET_CFGUPDATE : BQ27621_SOFT_RESET;

    BQ27621_write(BQ27621_REG_CTRL, cmd);

    BQ27621_read_flags(&status);

    do
    {
        syshal_time_delay_ms(25);
        BQ27621_read_flags(&status);
    }
    while (!!(status & BQ27621_FLAG_CFGUP) != active && --try);

    if (!try)
    {
        DEBUG_PR_ERROR("Timed out waiting for cfgupdate flag %d", active);
        return ERRORVAL;
    }

    if (limit - try > 3)
            DEBUG_PR_WARN("Cfgupdate %d, retries %lu", active, limit - try);

    return 0;
}

static inline int BQ27621_set_cfgupdate(void)
{
    int status = BQ27621_cfgupdate(true);
    if (ERRORVAL == status)
        DEBUG_PR_ERROR("Bus error on set_cfgupdate: %d", status);

    return status;
}

static inline int BQ27621_soft_reset(void)
{
    DEBUG_PR_TRACE("%s", __FUNCTION__);
    int status = BQ27621_cfgupdate(false);
    if (ERRORVAL == status)
        DEBUG_PR_ERROR("Bus error on soft_reset: %d", status);

    return status;
}

//////////////////////////////////////// Exposed functions ////////////////////////////////////////

int syshal_batt_init(void)
{
    syshal_gpio_init(GPIO_GPOUT);
    syshal_gpio_set_output_high(GPIO_GPOUT);

    number_of_charge_levels_zero = 0;

    //BQ27621_set_cfgupdate();
    if (syshal_i2c_is_device_ready(I2C_BATTERY, BQ27621_ADDR) != SYSHAL_I2C_NO_ERROR)
    {
        DEBUG_PR_ERROR("BQ27621 unresponsive");
        return SYSHAL_BATT_ERROR_DEVICE_UNRESPONSIVE;
    }

    syshal_time_delay_ms(1000);

    // Issue a soft reset if the ITPOR flag is set
    uint16_t flags = 0;
    int error;
    error = BQ27621_read_flags(&flags);
    if (flags & BQ27621_FLAG_ITPOR)
    {
        BQ27621_soft_reset();
        syshal_time_delay_ms(1000);
    }

    error = BQ27621_read_flags(&flags);

    // If the ITPOR flag has still not been cleared then the device is not responding
    if (flags & BQ27621_FLAG_ITPOR ||
        error < 0)
        return SYSHAL_BATT_ERROR_DEVICE_UNRESPONSIVE;

    // Wait for device to start giving stable SOC (state of charge) readings above 0
    uint8_t level = 0;
    int past_level = 0;
    uint32_t number_of_stable_levels = 0;

    while (level == 0 || number_of_stable_levels < 1000)
    {
        syshal_time_delay_ms(1); // Give the device time between requests to start-up
        int bytes_written = syshal_i2c_read_reg(I2C_BATTERY, BQ27621_ADDR, BQ27621_REG_STATE_OF_CHARGE, &level, sizeof(level));

        if (bytes_written == sizeof(level))
        {
            if (level != past_level)
            {
                number_of_stable_levels = 0;
            }
            else
            {
                if (level > 0)
                {
                    number_of_stable_levels++;
                }
                else
                {
                    number_of_stable_levels = 0;
                }
            }

            past_level = level;
        }
    }

    DEBUG_PR_TRACE("BQ27621 init");

    return SYSHAL_BATT_NO_ERROR;
}

// Return the battery temperature in tenths of degree Kelvin
/*uint16_t syshal_batt_temp(void)
{
    return BQ27621_read(BQ27621_REG_TEMP);
}

// Return the battery Voltage in millivolts
uint16_t syshal_batt_voltage(void)
{
    return BQ27621_read(BQ27621_REG_VOLT);
}*/

/**
 * @brief      Returns the percentage charge left in the battery
 *
 * @return     0 -> 100%
 */
int syshal_batt_level(void)
{
    uint16_t flags = 0;
    uint8_t level;

    // Has the device reset for some reason?
    int error = BQ27621_read_flags(&flags);

    if (error < 0)
        return SYSHAL_BATT_ERROR_DEVICE_UNRESPONSIVE;

    if (flags & BQ27621_FLAG_ITPOR)
    {
        DEBUG_PR_WARN("BQ27621 has reset, FLAGS: 0x%04X", flags);
        syshal_batt_init();
        BQ27621_read_flags(&flags);
        DEBUG_PR_WARN("BQ27621 after reset, FLAGS: 0x%04X", flags);
    }

    int status = syshal_i2c_read_reg(I2C_BATTERY, BQ27621_ADDR, BQ27621_REG_STATE_OF_CHARGE, &level, 1);

    if (0 == level)
    {
        number_of_charge_levels_zero++;
        DEBUG_PR_WARN("BQ27621 zero reading");
        if (number_of_charge_levels_zero < BQ27621_NUM_OF_ZERO_CHARGE_REQUIRED)
            return SYSHAL_BATT_ERROR_BUSY;
    }

    if (SYSHAL_I2C_ERROR_TIMEOUT == status)
        return SYSHAL_BATT_ERROR_TIMEOUT;
    else if (status <= 0)
        return SYSHAL_BATT_ERROR_DEVICE_UNRESPONSIVE;
    else
        return level;
}

/**
 * @brief      Is the battery currently in a charging state?
 *
 * @return     True if charging, false if not
 */
/*bool syshal_batt_charging(void)
{
    uint8_t data[2];
    syshal_i2c_read_reg(I2C_BATTERY, BQ27621_ADDR, BQ27621_REG_AVERAGE_POWER, data, 2);

    int16_t averagePower = ((int16_t)data[1] << 8) | ((int16_t)data[0] & 0xff);

    if (averagePower >= 0)
        return true;
    else
        return false;
}*/