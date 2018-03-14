/* config_if.c - Configuration interface abstraction layer. This is used
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

#include "config_if.h"
#include "syshal_usb.h"
#include "debug.h"

int (*config_if_transfer_priv)(uint8_t *, uint32_t);
int (*config_if_receive_priv)(uint8_t *, uint32_t);
bool (*config_if_peek_at_priv)(uint8_t *, uint32_t);
int (*config_if_available_priv)(void);

// Constants
int config_if_init(config_if_interface_t interface)
{
    if (interface == CONFIG_IF_USB)
    {
        config_if_transfer_priv = &syshal_usb_transfer;
        config_if_receive_priv = &syshal_usb_receive;
        config_if_peek_at_priv = &syshal_usb_peek_at;
        config_if_available_priv = &syshal_usb_available;

        return CONFIG_IF_NO_ERROR;
    }
    else if (interface == CONFIG_IF_BLE)
    {
        DEBUG_PR_ERROR("BLE NOT IMPLEMENTED %s()", __FUNCTION__);
        /*
        config_if_transfer_priv = &syshal_ble_transfer;
        config_if_receive_priv = &syshal_ble_receive;
        config_if_peek_at_priv = &syshal_ble_peek_at;
        config_if_available_priv = &syshal_ble_available;
        */

        return CONFIG_IF_ERROR_INVALID_INSTANCE;
    }
    else
    {
        return CONFIG_IF_ERROR_INVALID_INSTANCE;
    }
}

int config_if_transfer(uint8_t * data, uint32_t size)
{
    if (config_if_transfer_priv == NULL)
        return CONFIG_IF_ERROR_INVALID_INSTANCE;

    return config_if_transfer_priv(data, size);
}

int config_if_receive(uint8_t * data, uint32_t size)
{
    if (config_if_transfer_priv == NULL)
        return CONFIG_IF_ERROR_INVALID_INSTANCE;

    return config_if_receive_priv(data, size);
}

bool config_if_peek_at(uint8_t * byte, uint32_t location)
{
    if (config_if_transfer_priv == NULL)
        return CONFIG_IF_ERROR_INVALID_INSTANCE;

    return config_if_peek_at_priv(byte, location);
}

int config_if_available()
{
    if (config_if_transfer_priv == NULL)
        return CONFIG_IF_ERROR_INVALID_INSTANCE;

    return config_if_available_priv();
}