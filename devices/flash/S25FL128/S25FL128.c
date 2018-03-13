/* S25FL128.c - HAL implementation for S25FL128 flash memory device
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

#include <string.h>
#include <stdint.h>

#include "S25FL128.h"
#include "syshal_flash.h"
#include "syshal_spi.h"

/* Constants */

/* Macros */

#define MIN(x, y)  (x) < (y) ? (x) : (y)

/* Types */

/* Static Variables */

/* Keep track of the SPI device number for a given
 * logical flash drive.
 */
static uint32_t spi_device[S25FL128_MAX_DEVICES];

/* Local buffers for building SPI commands.  We
 * provision for 4 extra bytes to allow for the
 * command byte plus 3 bytes of addressing.
 */
static uint8_t  spi_tx_buf[S25FL128_PAGE_SIZE + 4];
static uint8_t  spi_rx_buf[S25FL128_PAGE_SIZE + 4];

/* Static Functions */

/*! \brief Obtain flash device's status register.
 *
 * \param spi_device[in] SPI device number for comms.
 * \param status[out] pointer for storing status register.
 *
 * \return Refer to syshal_spi error codes definitions.
 */
static int S25FL128_status(uint32_t spi_device, uint8_t *status)
{
    int ret;

    spi_tx_buf[0] = RDSR;
    spi_tx_buf[1] = 0;

    ret = syshal_spi_transfer(spi_device, spi_tx_buf, spi_rx_buf, 2);
    *status = spi_rx_buf[1];

    return ret;
}

/*! \brief Execute write enable command with flash device.
 *
 * \param spi_device[in] SPI device number for comms.
 *
 * \return Refer to syshal_spi error codes definitions.
 */
static int S25FL128_wren(uint32_t spi_device)
{
    spi_tx_buf[0] = WREN;

    return syshal_spi_transfer(spi_device, spi_tx_buf, spi_rx_buf, 1);
}

/*! \brief Execute erase sector command with flash device.
 *
 * Following command execution, this routine will busy wait until
 * the status register indicates it is no longer busy.  This
 * allows back-to-back operations to be performed without
 * first checking for a busy condition.
 *
 * \param spi_device[in] SPI device number for comms.
 * \param addr[in] Physical byte address in flash to erase.
 *
 * \return Refer to syshal_spi error codes definitions.
 */
static int S25FL128_erase_sector(uint32_t spi_device, uint32_t addr)
{
    int ret;
    uint8_t status;

    spi_tx_buf[0] = SE;
    spi_tx_buf[1] = addr >> 16;
    spi_tx_buf[2] = addr >> 8;
    spi_tx_buf[3] = addr;

    ret = syshal_spi_transfer(spi_device, spi_tx_buf, spi_rx_buf, 4);
    if (ret) return ret;

    /* Busy wait until the sector has erased */
    do
    {
        ret = S25FL128_status(spi_device, &status);
    } while ((status & RDSR_BUSY) && !ret);

    return ret;
}

/* Exported Functions */

/*! \brief Initialize flash drive.
 *
 * A logical flash drive is associated with the specified
 * SPI device instance number and the SPI driver for
 * this device is also initialized.
 *
 * \param drive[in] Logical drive number.
 * \param device[in] SPI device instance for subsequent comms.
 *
 * \return SYSHAL_FLASH_NO_ERROR on success.
 * \return SYSHAL_FLASH_ERROR_DEVICE on SPI initialization error.
 * \return SYSHAL_FLASH_ERROR_INVALID_DRIVE if the drive number is invalid.
 */
int syshal_flash_init(uint32_t drive, uint32_t device)
{
    if (drive >= S25FL128_MAX_DEVICES)
        return SYSHAL_FLASH_ERROR_INVALID_DRIVE;

    spi_device[drive] = device;

    if (syshal_spi_init(device))
        return SYSHAL_FLASH_ERROR_DEVICE;

    return SYSHAL_FLASH_NO_ERROR;
}

/*! \brief Terminate a flash drive.
 *
 * A logical flash drive is associated with the specified
 * SPI device instance number and the SPI driver for
 * this device is also terminated.
 *
 * \param drive[in] Logical drive number.
 *
 * \return SYSHAL_FLASH_NO_ERROR on success.
 * \return SYSHAL_FLASH_ERROR_DEVICE on SPI initialization error.
 * \return SYSHAL_FLASH_ERROR_INVALID_DRIVE if the drive number is invalid.
 */
int syshal_flash_term(uint32_t drive)
{
    if (drive >= S25FL128_MAX_DEVICES)
        return SYSHAL_FLASH_ERROR_INVALID_DRIVE;

    if (syshal_spi_term(spi_device[drive]))
        return SYSHAL_FLASH_ERROR_DEVICE;

    return SYSHAL_FLASH_NO_ERROR;
}

/*! \brief Erase flash sectors.
 *
 * The input address and size are both assumed to be sector
 * aligned.
 *
 * \param drive[in] Logical drive number.
 * \param address[in] Byte address in flash to erase.
 * \param size[in] Number of bytes in flash to erase.
 *
 * \return SYSHAL_FLASH_NO_ERROR on success.
 * \return SYSHAL_FLASH_ERROR_DEVICE on SPI error.
 * \return SYSHAL_FLASH_ERROR_INVALID_DRIVE if the drive number is invalid.
 */
int syshal_flash_erase(uint32_t drive, uint32_t address, uint32_t size)
{
    if (drive >= S25FL128_MAX_DEVICES)
        return SYSHAL_FLASH_ERROR_INVALID_DRIVE;

    /* Enable writes */
    if (S25FL128_wren(spi_device[drive]))
        return SYSHAL_FLASH_ERROR_DEVICE;

    /* Iteratively erase sectors */
    for (uint32_t i = 0; i < size; i += S25FL128_SECTOR_SIZE)
    {
        if (S25FL128_erase_sector(spi_device[drive], address))
            return SYSHAL_FLASH_ERROR_DEVICE;
        address += S25FL128_SECTOR_SIZE;
    }

    return SYSHAL_FLASH_NO_ERROR;
}

/*! \brief Write data to flash drive.
 *
 * It is not permitted to write across page boundaries with
 * write operations.  This will lead to unknown behaviour.
 *
 * \param drive[in] Logical drive number.
 * \param src[in] Source buffer pointer containing bytes to write.
 * \param address[in] Byte address in flash to write.
 * \param size[in] Number of bytes in flash to write.
 *
 * \return SYSHAL_FLASH_NO_ERROR on success.
 * \return SYSHAL_FLASH_ERROR_DEVICE on SPI error.
 * \return SYSHAL_FLASH_ERROR_INVALID_DRIVE if the drive number is invalid.
 */
int syshal_flash_write(uint32_t drive, const void *src, uint32_t address, uint32_t size)
{
    uint8_t status;

    if (drive >= S25FL128_MAX_DEVICES)
        return SYSHAL_FLASH_ERROR_INVALID_DRIVE;

    /* Enable writes */
    if (S25FL128_wren(spi_device[drive]))
        return SYSHAL_FLASH_ERROR_DEVICE;

    while (size > 0)
    {
        spi_tx_buf[0] = PP;
        spi_tx_buf[1] = address >> 16;
        spi_tx_buf[2] = address >> 8;
        spi_tx_buf[3] = address;

        uint16_t wr_size = MIN(size, S25FL128_PAGE_SIZE);
        memcpy(&spi_tx_buf[4], src, wr_size);
        if (syshal_spi_transfer(spi_device[drive], spi_tx_buf, spi_rx_buf, wr_size + 4))
            return SYSHAL_FLASH_ERROR_DEVICE;
        size -= wr_size;
        address += wr_size;

        /* Busy wait until the sector has erased */
        do
        {
            if (S25FL128_status(spi_device[drive], &status))
                return SYSHAL_FLASH_ERROR_DEVICE;
        } while (status & RDSR_BUSY);
    }

    return SYSHAL_FLASH_NO_ERROR;
}

/*! \brief Read data from flash drive.
 *
 * It is not permitted to read across page boundaries with
 * read operations.  This will lead to unknown behaviour.
 *
 * \param drive[in] Logical drive number.
 * \param dest[out] Destination buffer pointer to copy read data into.
 * \param address[in] Byte address in flash to read.
 * \param size[in] Number of bytes in flash to read.
 *
 * \return SYSHAL_FLASH_NO_ERROR on success.
 * \return SYSHAL_FLASH_ERROR_DEVICE on SPI error.
 * \return SYSHAL_FLASH_ERROR_INVALID_DRIVE if the drive number is invalid.
 */
int syshal_flash_read(uint32_t drive, void *dest, uint32_t address, uint32_t size)
{
    if (drive >= S25FL128_MAX_DEVICES)
        return SYSHAL_FLASH_ERROR_INVALID_DRIVE;

    memset(spi_tx_buf, 0, sizeof(spi_tx_buf));

    spi_tx_buf[0] = READ;

    while (size > 0)
    {
        spi_tx_buf[1] = address >> 16;
        spi_tx_buf[2] = address >> 8;
        spi_tx_buf[3] = address;

        uint16_t rd_size = MIN(size, S25FL128_PAGE_SIZE);

        if (syshal_spi_transfer(spi_device[drive], spi_tx_buf, spi_rx_buf, rd_size + 4))
            return SYSHAL_FLASH_ERROR_DEVICE;

        size -= rd_size;
        address += rd_size;
        memcpy(dest, &spi_rx_buf[4], rd_size);
        dest += rd_size;
    }

    return SYSHAL_FLASH_NO_ERROR;
}
