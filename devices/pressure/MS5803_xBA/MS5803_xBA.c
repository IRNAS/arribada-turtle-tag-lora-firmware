/* MS5803_xBA.c - Device driver for MS5803_xBA pressure sensors
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

#include "MS5803_xBA.h"
#include "syshal_i2c.h"
#include "syshal_time.h"
#include "syshal_pressure.h"
#include "bsp.h"
#include "debug.h"

uint16_t MS5803_xBA_coefficient[8];

int syshal_pressure_init(void)
{
    // Read PROM contents
    MS5803_xBA_read_prom(&MS5803_xBA_coefficient[0]);

    // The last 4 bits of the 7th coefficient form a CRC error checking code.
    uint8_t crc4_read = MS5803_xBA_coefficient[7];

    // Calculate the actual crc value
    uint8_t crc4_actual = MS5803_xBA_calculate_crc4(&MS5803_xBA_coefficient[0]);

    // Compare the calculated crc to the one read
    if (crc4_actual != crc4_read)
    {
        DEBUG_PR_TRACE("%s() CRC mismatch: crc4_read = 0x%02X, crc4_actual = 0x%02X", __FUNCTION__, crc4_read, crc4_actual);
        return SYSHAL_PRESSURE_ERROR_CRC_MISMATCH;
    }

    return SYSHAL_PRESSURE_NO_ERROR;
}

uint8_t MS5803_xBA_calculate_crc4(uint16_t prom[])
{
    uint32_t n_rem = 0; // crc reminder

    uint16_t crc_temp = prom[7]; // Save the end of the configuration data

    prom[7] &= 0xFF00; // Replace the 4 bit CRC with 0s

    for (uint32_t i = 0; i < 16; ++i)
    {
        // choose LSB or MSB
        if (i % 2 == 1)
            n_rem ^= (prom[i >> 1]) & 0x00FF;
        else
            n_rem ^= (prom[i >> 1] >> 8);

        for (uint32_t j = 8; j > 0; j--) // For each bit in a byte
        {
            if (n_rem & (0x8000))
                n_rem = (n_rem << 1) ^ 0x3000;
            else
                n_rem = (n_rem << 1);
        }
    }

    n_rem = (0x000F & (n_rem >> 12)); // final 4-bit reminder is CRC code
    prom[7] = crc_temp; // restore the crc_read to its original place

    return (n_rem ^ 0x00);
}

void MS5803_xBA_send_command(uint8_t command)
{
    syshal_i2c_transfer(I2C_PRESSURE, MS5803_I2C_ADDRESS, &command, sizeof(command));
}

void MS5803_xBA_read_prom(uint16_t prom[])
{
    for (uint32_t i = 0; i < 8; ++i)
    {
        uint8_t read_buffer[2];

        // The PROM starts at address 0xA0
        MS5803_xBA_send_command(CMD_PROM | (i * 2));
        syshal_i2c_receive(I2C_PRESSURE, MS5803_I2C_ADDRESS, read_buffer, 2);

        prom[i] = (((uint16_t)read_buffer[0] << 8) + read_buffer[1]);
    }
}

// Retrieve ADC measurement from the device.
// Select measurement type and precision
uint32_t MS5803_xBA_get_adc(uint8_t measurement, uint8_t precision)
{
    // Initiate a ADC read
    MS5803_xBA_send_command(CMD_ADC_CONV | measurement | precision);

    // Wait for conversion to complete
    syshal_time_delay_ms(1); // general delay
    switch (precision)
    {
        case CMD_ADC_256:  syshal_time_delay_ms(1); break;
        case CMD_ADC_512:  syshal_time_delay_ms(3); break;
        case CMD_ADC_1024: syshal_time_delay_ms(4); break;
        case CMD_ADC_2048: syshal_time_delay_ms(6); break;
        case CMD_ADC_4096: syshal_time_delay_ms(10); break;
    }

    // Issue an ADC read command
    MS5803_xBA_send_command(CMD_ADC_READ);

    // Read the ADC values
    uint8_t buffer[3];
    syshal_i2c_receive(I2C_PRESSURE, MS5803_I2C_ADDRESS, (uint8_t *) &buffer[0], sizeof(buffer));

    return ((uint32_t)buffer[0] << 16) + ((uint32_t)buffer[1] << 8) + buffer[2];
}