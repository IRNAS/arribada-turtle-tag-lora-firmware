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

static int (*config_if_func_send_priv)(uint8_t *, uint32_t);
static int (*config_if_func_receive_priv)(uint8_t *, uint32_t);

static config_if_backend_t backend_priv = CONFIG_IF_BACKEND_NOT_SET;

// Constants
int config_if_init(config_if_backend_t backend)
{
    if (backend_priv == backend)
        return CONFIG_IF_ERROR_ALREADY_CONFIGURED;

    if (backend == CONFIG_IF_BACKEND_USB)
    {
        config_if_func_send_priv = &syshal_usb_send;
        config_if_func_receive_priv = &syshal_usb_receive;

        syshal_usb_init();

        backend_priv = backend;

        return CONFIG_IF_NO_ERROR;
    }
    else if (backend == CONFIG_IF_BACKEND_BLE)
    {
        DEBUG_PR_ERROR("BLE NOT IMPLEMENTED %s()", __FUNCTION__);
        /*
        config_if_transfer_priv = &syshal_ble_transfer;
        config_if_receive_priv = &syshal_ble_receive;
        */

        backend_priv = backend;

        return CONFIG_IF_ERROR_INVALID_INSTANCE;
    }
    else
    {
        return CONFIG_IF_ERROR_INVALID_INSTANCE;
    }
}

int config_if_term(void)
{
    config_if_func_send_priv = NULL;
    config_if_func_receive_priv = NULL;

    if (backend_priv == CONFIG_IF_BACKEND_USB)
        syshal_usb_term();

    if (backend_priv == CONFIG_IF_BACKEND_BLE)
    {
        DEBUG_PR_ERROR("BLE NOT IMPLEMENTED %s()", __FUNCTION__);
        //syshal_ble_term();
    }

    backend_priv = CONFIG_IF_BACKEND_NOT_SET;

    return CONFIG_IF_NO_ERROR;
}

int config_if_send(uint8_t * data, uint32_t size)
{
    if (config_if_func_send_priv == NULL)
        return CONFIG_IF_ERROR_INVALID_INSTANCE;

    return config_if_func_send_priv(data, size);
}

int config_if_receive(uint8_t * data, uint32_t size)
{
    if (config_if_func_receive_priv == NULL)
        return CONFIG_IF_ERROR_INVALID_INSTANCE;

    return config_if_func_receive_priv(data, size);
}

/**
 * @brief      This function is called whenever a event occurs on the configured
 *             backend. This should be the user's application code to handle
 *             communication events
 *
 * @param[out] event  The event
 *
 * @return     Return error code
 */
__attribute__((weak)) int config_if_event_handler(config_if_event_t * event)
{
    ((void)(event)); // Remove unused variable compiler warning
    DEBUG_PR_WARN("%s Not implemented", __FUNCTION__);

    return CONFIG_IF_NO_ERROR;
}

/**
 * @brief      This function is called whenever a event occurs on the USB bus.
 *             This should be overriden by config_if.c
 *
 * @param[out] event  The event
 *
 * @return     Return error code
 */
int syshal_usb_event_handler(syshal_usb_event_t * event)
{
    // Do a manual member copy of the events. This is to avoid bugs arising from overlaying structures that later change
    config_if_event_t parsedEvent;
    parsedEvent.id = event->id;

    switch (event->id)
    {
        case CONFIG_IF_EVENT_SEND_COMPLETE:
            parsedEvent.send.buffer = event->send.buffer;
            parsedEvent.send.size = event->send.size;
            break;

        case CONFIG_IF_EVENT_RECEIVE_COMPLETE:
            parsedEvent.receive.buffer = event->receive.buffer;
            parsedEvent.receive.size = event->receive.size;
            break;

        case CONFIG_IF_EVENT_CONNECTED:
        case CONFIG_IF_EVENT_DISCONNECTED:
        default:
            break;
    }

    config_if_event_handler(&parsedEvent);

    return SYSHAL_USB_NO_ERROR;
}