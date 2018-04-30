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

    // The supported options by the LSM9DS1 device
    uint16_t valid_sample_rate_options[6] = {10, 50, 119, 238, 476, 952}; // Number of readings per second
    uint16_t valid_g_force_high_threshold_options[4] = {2, 4, 8, 16}; // Maximum g-force ±xg

    // Convert the configuration tag options to the closest supported option
    uint16_t sample_rate = find_closest_value_priv(axl_sample_rate_tag.contents.sample_rate, valid_sample_rate_options, sizeof(valid_sample_rate_options));
    uint16_t g_force_high_threshold = find_closest_value_priv(axl_g_force_high_threshold_tag.contents.threshold, valid_g_force_high_threshold_options, sizeof(valid_g_force_high_threshold_options));

    // Populate the accelerometer CTRL_REG6_XL register
    uint8_t reg_value = 0;

    switch (sample_rate)
    {
        case 10:
            reg_value |= (0x1 << LSM9D1_CTRL_REG6_XL_ODR_XL_POS);
            break;
        case 50:
            reg_value |= (0x2 << LSM9D1_CTRL_REG6_XL_ODR_XL_POS);
            break;
        case 119:
            reg_value |= (0x3 << LSM9D1_CTRL_REG6_XL_ODR_XL_POS);
            break;
        case 238:
            reg_value |= (0x4 << LSM9D1_CTRL_REG6_XL_ODR_XL_POS);
            break;
        case 476:
            reg_value |= (0x5 << LSM9D1_CTRL_REG6_XL_ODR_XL_POS);
            break;
        case 952:
            reg_value |= (0x6 << LSM9D1_CTRL_REG6_XL_ODR_XL_POS);
            break;
    }

    switch (g_force_high_threshold)
    {
        case 4:
            reg_value |= (0x2 << LSM9D1_CTRL_REG6_XL_FS_XL_POS);
            break;
        case 8:
            reg_value |= (0x3 << LSM9D1_CTRL_REG6_XL_FS_XL_POS);
            break;
        case 16:
            reg_value |= (0x1 << LSM9D1_CTRL_REG6_XL_FS_XL_POS);
            break;
            // Otherwise it'll be set to 2g (0x0 << LSM9D1_CTRL_REG6_XL_FS_XL_POS)
    }

    syshal_i2c_write_reg(I2C_AXL, LSM9D1_ADDR, LSM9D1_CTRL_REG6_XL, &reg_value, 1);

    return SYSHAL_AXL_NO_ERROR;
}

// Init the gyroscope
int syshal_gyro_init(void)
{
    // Fetch all required configuration tags
    int ret;

    sys_config_gyro_sample_rate_t gyro_sample_rate_tag;
    ret = sys_config_get(SYS_CONFIG_TAG_GYRO_SAMPLE_RATE, (void *) &gyro_sample_rate_tag.contents);
    if (ret < 0)
        return SYSHAL_GYRO_PROVISIONING_NEEDED;

    sys_config_gyro_degrees_per_second_t axl_gyro_degrees_per_second_tag;
    ret = sys_config_get(SYS_CONFIG_TAG_GYRO_DEGREES_PER_SECOND, (void *) &axl_gyro_degrees_per_second_tag.contents);
    if (ret < 0)
        return SYSHAL_GYRO_PROVISIONING_NEEDED;

    // The supported options by the LSM9DS1 device
    uint16_t valid_sample_rate_options[6] = {15, 60, 119, 238, 476, 952}; // Number of readings per second
    uint16_t valid_degrees_per_second_options[3] = {245, 500, 2000}; // Maximum degrees per second

    // Convert the configuration tag options to the closest supported option
    uint16_t sample_rate = find_closest_value_priv(gyro_sample_rate_tag.contents.sample_rate, valid_sample_rate_options, sizeof(valid_sample_rate_options));
    uint16_t degrees_per_second = find_closest_value_priv(axl_gyro_degrees_per_second_tag.contents.degrees_per_second, valid_degrees_per_second_options, sizeof(valid_degrees_per_second_options));

    // Populate the gyroscope CTRL_REG1_G register
    uint8_t reg_value = 0;

    switch (sample_rate)
    {
        case 15:
            reg_value |= (0x1 << LSM9D1_CTRL_REG1_G_ODR_G_POS);
            break;
        case 60:
            reg_value |= (0x2 << LSM9D1_CTRL_REG1_G_ODR_G_POS);
            break;
        case 119:
            reg_value |= (0x3 << LSM9D1_CTRL_REG1_G_ODR_G_POS);
            break;
        case 238:
            reg_value |= (0x4 << LSM9D1_CTRL_REG1_G_ODR_G_POS);
            break;
        case 476:
            reg_value |= (0x5 << LSM9D1_CTRL_REG1_G_ODR_G_POS);
            break;
        case 952:
            reg_value |= (0x6 << LSM9D1_CTRL_REG1_G_ODR_G_POS);
            break;
    }

    switch (degrees_per_second)
    {
        case 500:
            reg_value |= (0x1 << LSM9D1_CTRL_REG1_G_FS_G_POS);
            break;
        case 2000:
            reg_value |= (0x3 << LSM9D1_CTRL_REG1_G_FS_G_POS);
            break;
            // Otherwise we'll set it to 245 dps (0x0 << LSM9D1_CTRL_REG1_G_FS_G_POS)
    }

    syshal_i2c_write_reg(I2C_AXL, LSM9D1_ADDR, LSM9D1_CTRL_REG1_G, &reg_value, 1);

    return SYSHAL_AXL_NO_ERROR;
}