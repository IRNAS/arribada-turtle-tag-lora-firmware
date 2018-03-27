/* MS5803_14BA.c - Device driver for the MS5803_14BA pressure sensor
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

#include "MS5803_14BA.h"

static resolution_priv = CMD_ADC_4096; // The conversion resolution

// Taken from document AN520
static uint8_t MS_5803_CRC_priv(uint16_t * coefficients)
{
    uint16_t n_rem = 0; // crc reminder
    uint16_t crc_read;  // original value of the CRC

    crc_temp = coefficients[7]; // Save the end of the configuration data

    coefficients[7] = (0xFF00 & (coefficients[7])); // Replace the 4 bit CRC with 0s
    for (uint32_t i = 0; i < 16; i++)
    {
        // choose LSB or MSB
        if (i % 2 == 1)
            n_rem ^= (coefficients[i >> 1]) & 0x00FF;
        else
            n_rem ^= (coefficients[i >> 1] >> 8);

        for (uint32_t j = 8; j > 0; j--) // For each bit in a byte
        {
            if (n_rem & (0x8000))
                n_rem = (n_rem << 1) ^ 0x3000;
            else
                n_rem = (n_rem << 1);
        }
    }

    n_rem = (0x000F & (n_rem >> 12)); // final 4-bit reminder is CRC code
    coefficients[7] = crc_temp; // restore the crc_read to its original place

    return (n_rem ^ 0x00);
}

int syshal_pressure_init(void)
{
    // Read PROM contents
    uint16_t coefficients[8];
    syshal_i2c_read_reg(I2C_INSTANCE, MS5803_I2C_ADDRESS, CMD_PROM, &coefficients[0], sizeof(coefficients));

    // The last 4 bits of the 7th coefficient form a CRC error checking code.
    uint8_t crc4_read = coefficients[7];

    // Calculate the actual crc value
    crc4_actual = MS_5803_CRC_priv(&coefficients[0]);

    // Compare the calculated crc to the one read
    if (crc4_actual != crc4_read)
        return SYSHAL_PRESSURE_ERROR_CRC_MISMATCH;

    return SYSHAL_PRESSURE_NO_ERROR;
}

// See datasheet for details
int32_t syshal_pressure_get(void)
{
    // Retrieve ADC result
    int32_t D1 = getADCconversion(CMD_ADC_D1, resolution_priv);
    int32_t D2 = getADCconversion(CMD_ADC_D2, resolution_priv);

    // working variables
    int32_t TEMP, dT;
    int64_t OFF, SENS, T2, OFF2, SENS2;

    /* Calculate first order coefficients */

    // Difference between actual and reference temperature
    // dT = D2 - C5 * 2^8
    dT = D2 - ((int32_t)coefficient[5] << 8);

    // Actual temperature
    // TEMP = 2000 + dT * C6 / 2^23
    TEMP = (((int64_t)dT * coefficient[6]) >> 23) + 2000;

    // Offset at actual temperature
    // OFF = C2 * 2^16 + (C4 * dT ) / 2^7
    OFF = ((int64_t)coefficient[2] << 16) + (((coefficient[4] * (int64_t)dT)) >> 7);

    // Sensitivity at actual temperature
    // SENS = C1 * 2^15 + ( C3 * dT ) / 2^8
    SENS = ((int64_t)coefficient[1] << 15) + (((coefficient[3] * (int64_t)dT)) >> 8);

    /* Calculate second order coefficients */

    if (TEMP < 2000) // If temp is below 20.0C
    {
        // T2 = 3 * dT^2 / 2^33
        T2 = 3 * (((int64_t)dT * dT) >> 33);

        // OFF2 = 3 * (TEMP - 2000) ^ 2 / 2
        OFF2 = 3 * ((TEMP - 2000) * (TEMP - 2000)) / 2;

        // SENS2 = 3 * (TEMP - 2000) ^ 2 / 2^3
        SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 8;

        if (TEMP < -1500) // If temp is below -15.0C
        {
            // OFF2 = OFF2 + 7 * (TEMP + 1500)^2
            OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));

            // SENS2 = SENS2 + 4 * (TEMP + 1500)^2
            SENS2 = SENS2 + 4 * ((TEMP + 1500) * (TEMP + 1500));
        }
    }
    else // If TEMP is above 20.0C
    {
        // T2 = 7 * dT^2 / 2^37
        T2 = 7 * ((uint64_t)dT * dT) / pow(2, 37);

        // OFF2 = 1 * (TEMP - 2000)^2 / 2^4
        OFF2 = ((TEMP - 2000) * (TEMP - 2000)) / 16;

        // SENS2 = 0
        SENS2 = 0;
    }

    /* Merge first and second order coefficients */

    // TEMP = TEMP - T2
    TEMP = TEMP - T2;

    // OFF = OFF - OFF2
    OFF = OFF - OFF2;

    // SENS = SENS - SENS2
    SENS = SENS - SENS2;

    /* Calculate pressure */

    // P = (D1 * SENS / 2^21 - OFF) / 2^15
    int32_t pressure = (((SENS * D1) / 2097152 ) - OFF) / 32768;

    return pressure;
}