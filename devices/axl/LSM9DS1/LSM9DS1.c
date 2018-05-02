/* LSM9DS1.c - HAL for accelerometer device
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

#include "LSM9DS1.h"
#include "syshal_gpio.h"
#include "syshal_i2c.h"
#include "bsp.h"

// Internal variables used to sleep and wake the device
static uint8_t LSM9D1_CTRL_REG6_XL_wake_state;
static uint8_t LSM9D1_CTRL_REG6_XL_power_down_state;

// Init the accelerometer
int syshal_axl_init(void)
{
    // Fetch all required configuration tags
    int ret;

    sys_config_axl_sample_rate_t axl_sample_rate_tag;
    ret = sys_config_get(SYS_CONFIG_TAG_AXL_SAMPLE_RATE, (void *) &axl_sample_rate_tag.contents);
    if (ret < 0)
        return SYSHAL_AXL_PROVISIONING_NEEDED;

    sys_config_axl_g_force_high_threshold_t axl_g_force_high_threshold_tag;
    ret = sys_config_get(SYS_CONFIG_TAG_AXL_G_FORCE_HIGH_THRESHOLD, (void *) &axl_g_force_high_threshold_tag.contents);
    if (ret < 0)
        return SYSHAL_AXL_PROVISIONING_NEEDED;

    // Enable the Accelerometer axis
    uint8_t reg_value = LSM9D1_CTRL_REG5_XL_ZEN_XL | LSM9D1_CTRL_REG5_XL_YEN_XL | LSM9D1_CTRL_REG5_XL_XEN_XL;
    syshal_i2c_write_reg(I2C_AXL, LSM9D1_ADDR, LSM9D1_CTRL_REG5_XL, &reg_value, 1);

    // The supported options by the LSM9DS1 device
    uint16_t valid_sample_rate_options[6] = {10, 50, 119, 238, 476, 952}; // Number of readings per second

    // Convert the configuration tag options to the closest supported option
    uint16_t sample_rate = find_closest_value_priv(axl_sample_rate_tag.contents.sample_rate, valid_sample_rate_options, sizeof(valid_sample_rate_options));
    uint16_t g_force_high_threshold = axl_g_force_high_threshold_tag.contents.threshold;

    // Populate the accelerometer CTRL_REG6_XL register
    reg_value = 0;

    switch (sample_rate)
    {
        case 10:
            reg_value |= LSM9D1_CTRL_REG6_XL_ODR_XL_10HZ;
            break;
        case 50:
            reg_value |= LSM9D1_CTRL_REG6_XL_ODR_XL_50HZ;
            break;
        case 119:
            reg_value |= LSM9D1_CTRL_REG6_XL_ODR_XL_119HZ;
            break;
        case 238:
            reg_value |= LSM9D1_CTRL_REG6_XL_ODR_XL_238HZ;
            break;
        case 476:
            reg_value |= LSM9D1_CTRL_REG6_XL_ODR_XL_476HZ;
            break;
        case 952:
            reg_value |= LSM9D1_CTRL_REG6_XL_ODR_XL_952HZ;
            break;
    }

    reg_value |= LSM9D1_CTRL_REG6_XL_FS_XL_4G;


    LSM9D1_CTRL_REG6_XL_wake_state = reg_value;
    LSM9D1_CTRL_REG6_XL_power_down_state = reg_value & (~LSM9D1_CTRL_REG6_XL_ODR_XL_MASK);
    syshal_i2c_write_reg(I2C_AXL, LSM9D1_ADDR, LSM9D1_CTRL_REG6_XL, &LSM9D1_CTRL_REG6_XL_wake_state, 1);

    return SYSHAL_AXL_NO_ERROR;
}

int syshal_axl_sleep(void)
{
    syshal_i2c_write_reg(I2C_AXL, LSM9D1_ADDR, LSM9D1_CTRL_REG6_XL, &LSM9D1_CTRL_REG6_XL_power_down_state, 1);

    return SYSHAL_AXL_NO_ERROR;
}

int syshal_axl_wake(void)
{
    syshal_i2c_write_reg(I2C_AXL, LSM9D1_ADDR, LSM9D1_CTRL_REG6_XL, &LSM9D1_CTRL_REG6_XL_wake_state, 1);

    return SYSHAL_AXL_NO_ERROR;
}