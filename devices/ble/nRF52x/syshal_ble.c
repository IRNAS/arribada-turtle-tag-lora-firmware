/* syshal_ble.c - HAL for ble device
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

#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#include "syshal_ble.h"
#include "syshal_spi.h"
#include "nRF52x_regs.h"
#include "syshal_time.h"
#include "bsp.h"
#include "syshal_gpio.h"
#include "debug.h"

/* Macros */

#define MIN(x, y)  (x) < (y) ? (x) : (y)
#define SPI_BUS_DELAY_MS (20)

/* Private variables */

static uint8_t      int_enable = 0x00;
static uint8_t      xfer_buffer[SYSHAL_BLE_MAX_BUFFER_SIZE + 1];
static uint32_t     spi_device;
static uint8_t   *  rx_buffer_pending = NULL;

static uint32_t     time_of_last_transfer = 0;

static uint16_t     rx_buffer_pending_size = 0;

static uint8_t   *  tx_buffer_pending = NULL;
static uint16_t     tx_bytes_to_transmit;
static uint16_t     tx_bytes_sent_to_nrf;
static uint16_t     tx_bytes_transmitted_over_ble;
static uint16_t     tx_last_data_length;

static uint16_t     rx_fifo_size = 0;
static uint16_t     tx_fifo_size = 0;
static bool         gatt_connected = false;
static bool         fw_update_pending = false;

/* Private functions */

static int read_register(uint16_t address, uint8_t * data, uint16_t size)
{
    int ret;

    // Request a read of len, at given addr as follows:
    // Byte 0 = address
    // Byte 1 = len[1]
    // Byte 2 = len[0]

    memset(xfer_buffer, 0, sizeof(xfer_buffer));
    xfer_buffer[0] = address;

    xfer_buffer[1] = size & 0x00FF;
    xfer_buffer[2] = (size >> 8) & 0x00FF;

    DEBUG_PR_TRACE("Transfering to nRF52: 0x%02X %d", xfer_buffer[0], size);

    while (syshal_time_get_ticks_ms() - time_of_last_transfer < SPI_BUS_DELAY_MS)
    {}

    syshal_gpio_set_output_low(GPIO_SPI1_CS_BT);
    ret = syshal_spi_transfer(spi_device, xfer_buffer, xfer_buffer, 3);
    syshal_gpio_set_output_high(GPIO_SPI1_CS_BT);
    time_of_last_transfer = syshal_time_get_ticks_ms();

    if (ret)
        return SYSHAL_BLE_ERROR_COMMS;

    // Wait for the BLE device to populate it's read buffer
    while (syshal_time_get_ticks_ms() - time_of_last_transfer < SPI_BUS_DELAY_MS)
    {}

    syshal_gpio_set_output_low(GPIO_SPI1_CS_BT);
    ret = syshal_spi_transfer(spi_device, xfer_buffer, xfer_buffer, size);
    syshal_gpio_set_output_high(GPIO_SPI1_CS_BT);
    time_of_last_transfer = syshal_time_get_ticks_ms();

    if (ret)
        return SYSHAL_BLE_ERROR_COMMS;

    DEBUG_PR_TRACE("Received from nRF52");
    for (uint32_t i = 0; i < size; ++i)
        printf("%02X ", xfer_buffer[i]);
    printf("\r\n");

    memcpy(data, &xfer_buffer[0], size);
    return SYSHAL_BLE_NO_ERROR;
}

static int write_register(uint16_t address, uint8_t * data, uint16_t size)
{
    int ret;

    xfer_buffer[0] = address | NRF52_SPI_WRITE_NOT_READ_ADDR;
    memcpy(&xfer_buffer[1], data, size);

    DEBUG_PR_TRACE("write_register(0x%02X, *data, %d)", address, size);
    for (uint32_t i = 0; i < size + 1; ++i)
        printf("%02X ", xfer_buffer[i]);
    printf("\r\n");

    while (syshal_time_get_ticks_ms() - time_of_last_transfer < SPI_BUS_DELAY_MS)
    {}

    syshal_gpio_set_output_low(GPIO_SPI1_CS_BT);
    ret = syshal_spi_transfer(spi_device, xfer_buffer, xfer_buffer, size + 1);
    syshal_gpio_set_output_high(GPIO_SPI1_CS_BT);

    time_of_last_transfer = syshal_time_get_ticks_ms();

    if (ret)
        return SYSHAL_BLE_ERROR_COMMS;

    return SYSHAL_BLE_NO_ERROR;
}

/* Exported functions */

int syshal_ble_init(uint32_t comms_device)
{
    spi_device = comms_device;
    uint32_t version;

    syshal_gpio_init(GPIO_SPI1_CS_BT);
    syshal_gpio_set_output_high(GPIO_SPI1_CS_BT);

    time_of_last_transfer = syshal_time_get_ticks_ms();

    /* Keep attempting to read the softdevice version until we either timeout or get the correct version */
    /* This is used to determine whether or not our bluetooth device has booted */
    // TODO: implement timeout
    uint16_t soft_dev_version = 0;
    while (soft_dev_version == 0x0000 ||
           soft_dev_version == 0xFFFF)
    {
        if (read_register(NRF52_REG_ADDR_SOFT_DEV_VERSION, (uint8_t *)&soft_dev_version, sizeof(soft_dev_version)))
            return SYSHAL_BLE_ERROR_NOT_DETECTED;
    }

    /* Read version to make sure the device is present */
    if (syshal_ble_get_version(&version))
        return SYSHAL_BLE_ERROR_NOT_DETECTED;
    DEBUG_PR_TRACE("NRF52 version = %08lX", version);

    /* Get the TX FIFO size */
    if (read_register(NRF52_REG_ADDR_TX_FIFO_SIZE, (uint8_t *)&tx_fifo_size, sizeof(tx_fifo_size)))
        return SYSHAL_BLE_ERROR_NOT_DETECTED;
    DEBUG_PR_TRACE("NRF52 TX FIFO Size = %u bytes", tx_fifo_size);

    /* Get the RX FIFO size */
    if (read_register(NRF52_REG_ADDR_RX_FIFO_SIZE, (uint8_t *)&rx_fifo_size, sizeof(rx_fifo_size)))
        return SYSHAL_BLE_ERROR_NOT_DETECTED;
    DEBUG_PR_TRACE("NRF52 RX FIFO Size = %u bytes", rx_fifo_size);

    /* Get the TX data length */
    uint16_t tx_buffer_pending_size;
    if (read_register(NRF52_REG_ADDR_TX_DATA_LENGTH, (uint8_t *)&tx_buffer_pending_size, sizeof(tx_buffer_pending_size)))
        return SYSHAL_BLE_ERROR_NOT_DETECTED;
    DEBUG_PR_TRACE("NRF52 TX data length = %u bytes", tx_buffer_pending_size);

    /* Get the RX data length */
    if (read_register(NRF52_REG_ADDR_RX_DATA_LENGTH, (uint8_t *)&rx_buffer_pending_size, sizeof(rx_buffer_pending_size)))
        return SYSHAL_BLE_ERROR_NOT_DETECTED;
    DEBUG_PR_TRACE("NRF52 RX data length = %u bytes", rx_buffer_pending_size);

    rx_buffer_pending_size = 0;

    /* Enable data port related interrupts */
    int_enable = NRF52_INT_TX_DATA_SENT | NRF52_INT_RX_DATA_READY;
    int ret = write_register(NRF52_REG_ADDR_INT_ENABLE, &int_enable, sizeof(int_enable));
    DEBUG_PR_TRACE("NRF52 Enabling interrupt on TX sent and RX data ready");

    /* Start the nRF52 GATT Server */
    uint8_t mode = NRF52_MODE_GATT_SERVER;
    write_register(NRF52_REG_ADDR_MODE, &mode, sizeof(mode));
    DEBUG_PR_TRACE("NRF52 start GATT server");

    return ret;
}

int syshal_ble_term(void)
{
    /* Deep sleep the nRF52 device */
    uint8_t mode = NRF52_MODE_DEEP_SLEEP;
    write_register(NRF52_REG_ADDR_MODE, &mode, sizeof(mode));

    // Clear any pending transfers
    rx_buffer_pending = NULL;
    tx_buffer_pending = NULL;

    return SYSHAL_BLE_NO_ERROR;
}

int syshal_ble_set_mode(syshal_ble_mode_t mode)
{
    int ret = write_register(NRF52_REG_ADDR_MODE, (uint8_t *)&mode, sizeof(uint8_t));

    if (!ret)
    {
        if (mode == SYSHAL_BLE_MODE_FW_UPGRADE)
            fw_update_pending = true;
        else
            fw_update_pending = false;
        if (mode != SYSHAL_BLE_MODE_GATT_SERVER &&
            mode != SYSHAL_BLE_MODE_GATT_CLIENT)
            gatt_connected = false;
    }
    return ret;
}

int syshal_ble_get_mode(syshal_ble_mode_t * mode)
{
    uint8_t rd_mode;
    int ret;

    ret = read_register(NRF52_REG_ADDR_MODE, &rd_mode, sizeof(rd_mode));
    *mode = (syshal_ble_mode_t)rd_mode;

    return ret;
}

int syshal_ble_get_version(uint32_t * version)
{
    int ret;
    uint16_t app_version, soft_dev_version;
    ret = read_register(NRF52_REG_ADDR_APP_VERSION, (uint8_t *)&app_version, sizeof(app_version));
    ret |= read_register(NRF52_REG_ADDR_SOFT_DEV_VERSION, (uint8_t *)&soft_dev_version, sizeof(soft_dev_version));
    *version = ((uint32_t) app_version << 16) | soft_dev_version;

    return ret;
}

int syshal_ble_config_fw_upgrade(syshal_ble_fw_upgrade_type_t type, uint32_t size, uint32_t crc)
{
    int ret;
    ret = write_register(NRF52_REG_ADDR_FW_UPGRADE_SIZE, (uint8_t *)&size, sizeof(size));
    ret |= write_register(NRF52_REG_ADDR_FW_UPGRADE_TYPE, (uint8_t *)&type, sizeof(uint8_t));
    ret |= write_register(NRF52_REG_ADDR_FW_UPGRADE_CRC, (uint8_t *)&crc, sizeof(crc));
    return ret;
}

int syshal_ble_reset(void)
{
    uint8_t mode = NRF52_MODE_RESET;
    return write_register(NRF52_REG_ADDR_MODE, &mode, sizeof(mode));
}

int syshal_ble_send(uint8_t * buffer, uint32_t size)
{
    /* Don't allow a transmit if there is already one pending */
    if (tx_buffer_pending)
        return SYSHAL_BLE_ERROR_TRANSMIT_PENDING;

    /* Just keep track of the user buffer, the actual transmit will
     * happen as part of the "tick" function.
     */
    tx_bytes_to_transmit = size;
    tx_bytes_sent_to_nrf = 0;
    tx_bytes_transmitted_over_ble = 0;
    tx_last_data_length = 0;

    tx_buffer_pending = buffer;

    return SYSHAL_BLE_NO_ERROR;
}

int syshal_ble_receive(uint8_t * buffer, uint32_t size)
{
    /* Don't allow a receive if there is already one pending */
    if (rx_buffer_pending)
        return SYSHAL_BLE_ERROR_RECEIVE_PENDING;

    /* Just keep track of the user buffer, the actual receive will
     * happen as part of the "tick" function.
     */
    rx_buffer_pending = buffer;
    rx_buffer_pending_size = size;

    return SYSHAL_BLE_NO_ERROR;
}

int syshal_ble_tick(void)
{
    int ret = SYSHAL_BLE_NO_ERROR;
    uint8_t int_status;

    /* Read interrupt status register */
    ret = read_register(NRF52_REG_ADDR_INT_STATUS, &int_status, sizeof(int_status));
    if (ret)
        goto done;

    if ((int_status & NRF52_INT_GATT_CONNECTED) && !gatt_connected)
    {
        gatt_connected = true;
        syshal_ble_event_t event =
        {
            .error = SYSHAL_BLE_NO_ERROR,
            .event_id = SYSHAL_BLE_EVENT_CONNECTED
        };
        syshal_ble_event_handler(&event);
    }
    else if ((int_status & NRF52_INT_GATT_CONNECTED) == 0 && gatt_connected)
    {
        gatt_connected = false;
        syshal_ble_event_t event =
        {
            .error = SYSHAL_BLE_NO_ERROR,
            .event_id = SYSHAL_BLE_EVENT_DISCONNECTED
        };
        syshal_ble_event_handler(&event);
    }

    if ((int_status & NRF52_INT_ERROR_INDICATION))
    {
        uint8_t error_indication;
        ret = read_register(NRF52_REG_ADDR_ERROR_CODE, (uint8_t *)&error_indication, sizeof(error_indication));
        if (ret)
            goto done;

        /* This will abort any pending FW update */
        fw_update_pending = false;
        syshal_ble_event_t event =
        {
            .error = (uint32_t) - error_indication,
            .event_id = SYSHAL_BLE_EVENT_ERROR_INDICATION
        };
        syshal_ble_event_handler(&event);
    }

    if ((int_status & NRF52_INT_FLASH_PROGRAMMING_DONE) && fw_update_pending)
    {
        fw_update_pending = false;
        syshal_ble_event_t event =
        {
            .error = SYSHAL_BLE_NO_ERROR,
            .event_id = SYSHAL_BLE_EVENT_FW_UPGRADE_COMPLETE
        };
        syshal_ble_event_handler(&event);
    }

    /* Check for any pending data to receive from the nRF52x's FIFO */
    if (rx_buffer_pending)
    {
        uint16_t length;
        ret = read_register(NRF52_REG_ADDR_RX_DATA_LENGTH, (uint8_t *)&length, sizeof(length));
        if (ret)
            goto done;

        /* Check FIFO occupancy size */
        if (length)
        {
            /* Copy as many bytes as are available upto the limit of the user's
             * pending buffer size.
             */
            uint16_t num_read_bytes = MIN(length, rx_buffer_pending_size);
            ret = read_register(NRF52_REG_ADDR_RX_DATA_PORT, rx_buffer_pending, num_read_bytes);
            if (ret)
                goto done;

            syshal_ble_event_t event =
            {
                .error = SYSHAL_BLE_NO_ERROR,
                .event_id = SYSHAL_BLE_EVENT_RECEIVE_COMPLETE,
                .receive_complete = {
                    .length = num_read_bytes
                }
            };

            /* Reset pending buffer */
            rx_buffer_pending = NULL;
            syshal_ble_event_handler(&event);
        }
    }

    /* Check for any pending send operations */
    if (tx_buffer_pending)
    {
        uint16_t length;
        ret = read_register(NRF52_REG_ADDR_TX_DATA_LENGTH, (uint8_t *)&length, sizeof(length));
        if (ret)
            goto done;

        /* Check to see if any data has been transmitted over BLE */
        if (length < tx_last_data_length)
        {
            tx_bytes_transmitted_over_ble += tx_last_data_length - length; /* Calculate how many bytes were transmitted */
        }

        tx_last_data_length = length;

        /* Has our transmission completed? */
        if (tx_bytes_transmitted_over_ble >= tx_bytes_to_transmit)
        {
            /* Signal send complete */
            syshal_ble_event_t event =
            {
                .error = SYSHAL_BLE_NO_ERROR,
                .event_id = SYSHAL_BLE_EVENT_SEND_COMPLETE,
                .send_complete = {
                    .length = tx_bytes_to_transmit
                }
            };

            /* Reset pending buffer */
            tx_buffer_pending = NULL;
            syshal_ble_event_handler(&event);
        }
        else
        {
            /* Do we still have data to transfer to the nRF? */
            if (tx_bytes_sent_to_nrf < tx_bytes_to_transmit)
            {
                /* Do we have room in the TX FIFO? */
                uint16_t tx_buffer_free_space = tx_fifo_size - length;

                DEBUG_PR_TRACE("tx_buffer_free_space: %u", tx_buffer_free_space);
                DEBUG_PR_TRACE("tx_fifo_size: %u", tx_fifo_size);
                DEBUG_PR_TRACE("length: %u", length);

                while (tx_bytes_sent_to_nrf < tx_bytes_to_transmit)
                {
                    /* There are now three conditions for sending data
                     * 1) We transmit the last of our data              (tx_bytes_to_transmit - tx_bytes_sent_to_nrf)
                     * 2) We transmit up until we max out the TX FIFO   (tx_buffer_free_space)
                     * 3) We transmit one full SPI packet size          (NRF52_SPI_DATA_PORT_SIZE)
                     */
    
                    uint16_t bytes_to_send = MIN( MIN(tx_buffer_free_space, NRF52_SPI_DATA_PORT_SIZE), tx_bytes_to_transmit - tx_bytes_sent_to_nrf);
    
                    ret = write_register(NRF52_REG_ADDR_TX_DATA_PORT, tx_buffer_pending, bytes_to_send);
                    if (ret)
                        goto done;
    
                    tx_last_data_length += bytes_to_send;
                    tx_bytes_sent_to_nrf += bytes_to_send;
                    tx_buffer_free_space -= bytes_to_send;
                }
            }

        }

    }

done:
    return ret;
}

__attribute__((weak)) void syshal_ble_event_handler(syshal_ble_event_t * event)
{
    (void)event;
}
