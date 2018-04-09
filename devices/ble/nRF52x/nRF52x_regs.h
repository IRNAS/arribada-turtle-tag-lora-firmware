/* nRF52x_regs.h - SPI register definitions for NRF52x
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

#ifndef _NRF52x_REGS_H_
#define _NRF52x_REGS_H_

/* Constants */

#define NRF52_SPI_DATA_PORT_SIZE            1024
#define NRF52_SPI_WRITE_NOT_READ_ADDR       0x80

/* NRF52 SPI register map */
#define NRF52_REG_ADDR_APP_VERSION          0x00
#define NRF52_REG_ADDR_SOFT_DEV_VERSION     0x01
#define NRF52_REG_ADDR_MODE                 0x02
#define NRF52_REG_ADDR_FW_UPGRADE_SIZE      0x10
#define NRF52_REG_ADDR_FW_UPGRADE_CRC       0x11
#define NRF52_REG_ADDR_FW_UPGRADE_TYPE      0x12
#define NRF52_REG_ADDR_INT_STATUS           0x21
#define NRF52_REG_ADDR_INT_ENABLE           0x22
#define NRF52_REG_ADDR_ERROR_CODE           0x23
#define NRF52_REG_ADDR_OWN_UUID             0x30
#define NRF52_REG_ADDR_TARGET_UUID          0x31
#define NRF52_REG_ADDR_BEACON_INTERVAL      0x40
#define NRF52_REG_ADDR_BEACON_PAYLOAD       0x41
#define NRF52_REG_ADDR_SCAN_RESPONSE        0x42
#define NRF52_REG_ADDR_TX_DATA_PORT         0x60
#define NRF52_REG_ADDR_TX_DATA_LENGTH       0x61
#define NRF52_REG_ADDR_RX_DATA_PORT         0x70
#define NRF52_REG_ADDR_RX_DATA_LENGTH       0x71

#define NRF52_MODE_RESET                    0x07

#define NRF52_INT_TX_DATA_SENT              0x01
#define NRF52_INT_RX_DATA_READY             0x02
#define NRF52_INT_GATT_CONNECTED            0x04
#define NRF52_INT_FLASH_PROGRAMMING_DONE    0x08
#define NRF52_INT_ERROR_INDICATION          0x10

#define NRF52_ERROR_NONE                    0
#define NRF52_ERROR_CRC                     1
#define NRF52_ERROR_TIMEOUT                 2
#define NRF52_ERROR_LENGTH                  3
#define NRF52_ERROR_FW_TYPE                 4

#endif /* _NRF52x_REGS_H_ */
