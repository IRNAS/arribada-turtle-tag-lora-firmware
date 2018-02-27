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

#include "BQ27621.h"
#include "syshal_batt.h"

static I2C_t I2C_handle;

void syshal_batt_init(I2C_t instance)
{
    I2C_handle = instance;
}

// Return the battery temperature in tenths of degree Kelvin
uint16_t syshal_batt_temp(void)
{
    uint8_t data[BQ27621_REG_TEMP_SIZE];
    uint32_t length = syshal_i2c_read_reg(I2C_handle, BQ27621_ADDR, BQ27621_REG_TEMP, data, BQ27621_REG_TEMP_SIZE);

    if (2 == length)
        return (data[0] << 8) | (data[1] & 0xff);
    else
        return 0;
}

// Return the battery Voltage in millivolts
uint16_t syshal_batt_voltage(void)
{
    uint8_t data[BQ27621_REG_VOLT_SIZE];
    uint32_t length = syshal_i2c_read_reg(I2C_handle, BQ27621_ADDR, BQ27621_REG_VOLT, data, BQ27621_REG_VOLT_SIZE);

    if (2 == length)
        return (data[0] << 8) | (data[1] & 0xff);
    else
        return 0;
}

