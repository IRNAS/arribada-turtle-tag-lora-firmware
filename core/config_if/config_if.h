/* config_if.h - Configuration interface abstraction layer. This is used
 * to homogenise the USB and BLE syshals for seamless switching between them
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

#ifndef _CONFIG_IF_H_
#define _CONFIG_IF_H_

#include <stdint.h>
#include <stdbool.h>

// Constants
#define CONFIG_IF_NO_ERROR                0
#define CONFIG_IF_ERROR_INVALID_SIZE     -1
#define CONFIG_IF_ERROR_INVALID_INSTANCE -2
#define CONFIG_IF_ERROR_BUSY             -3
#define CONFIG_IF_ERROR_TIMEOUT          -4
#define CONFIG_IF_ERROR_DEVICE           -5

typedef enum
{
    CONFIG_IF_USB,
    CONFIG_IF_BLE,
} config_if_interface_t;

int config_if_init(config_if_interface_t interface);
int config_if_transfer(uint8_t * data, uint32_t size);
int config_if_receive(uint8_t * data, uint32_t size);
bool config_if_peek_at(uint8_t * byte, uint32_t location);
int config_if_available();

#endif /* _CONFIG_IF_H_ */