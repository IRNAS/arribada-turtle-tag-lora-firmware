/* syshal_firmware.C - HAL for writing firmware images to MCU FLASH
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

#include "syshal_firmware.h"
#include "stm32f0xx_hal.h"
#include "bsp.h"
#include "syshal_gpio.h"

#define FLASH_START_ADDR (0x8000000)

// HAL to SYSHAL error code mapping table
static int hal_error_map[] =
{
    SYSHAL_FIRMWARE_NO_ERROR,
    SYSHAL_FIRMWARE_ERROR_DEVICE,
    SYSHAL_FIRMWARE_ERROR_BUSY,
    SYSHAL_FIRMWARE_ERROR_TIMEOUT,
};

static uint32_t writing_address; // The FLASH address we're currently writing to
static uint32_t bytes_remaining; // Number of buffer.bytes yet to be written to FLASH

static union
{
    uint32_t word;
    uint8_t bytes[4];
} buffer;

__attribute__ ((section (".ramfunc"))) int syshal_firmware_prepare(void)
{
    __disable_irq(); // Don't allow any interrupts to interrupt us

    writing_address = FLASH_START_ADDR;
    bytes_remaining = 0;

    HAL_FLASH_Unlock(); // Unlock the Flash to enable the flash control register access

    // Erase entire flash
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t page_error;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_MASSERASE;

    int ret = HAL_FLASHEx_Erase(&EraseInitStruct, &page_error);

    return hal_error_map[ret];
}

__attribute__ ((section (".ramfunc"))) int syshal_firmware_write(uint8_t * data, uint32_t size)
{
    // Iterate through every discrete byte
    for (uint32_t i = 0; i < size; ++i)
    {
        // Fill up blocks of 32 bits
        buffer.bytes[bytes_remaining] = data[i];
        bytes_remaining++;

        // If we have a full 32 bits, write it to FLASH
        if (bytes_remaining == 4)
        {
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, writing_address, buffer.word);
            writing_address += 4;
            bytes_remaining = 0;
        }
    }

    return SYSHAL_FIRMWARE_NO_ERROR;
}

__attribute__ ((section (".ramfunc"))) int syshal_firmware_flush(void)
{
    // Flush any remaining data to the FLASH
    if (bytes_remaining)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, writing_address, buffer.word);
        writing_address += 4;
        bytes_remaining = 0;
    }

    HAL_FLASH_Lock(); // Lock the FLASH to disable the flash control register access

    return SYSHAL_FIRMWARE_NO_ERROR;
}