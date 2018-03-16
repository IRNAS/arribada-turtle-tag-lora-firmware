/* syshal_usb.c - HAL for USB
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

#include "bsp.h"
#include "syshal_usb.h"
#include "usbd_conf.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_vendor.h"
#include "ring_buffer.h"
#include "debug.h"

// Internal variables

// HAL to SYSHAL error code mapping table
static int hal_error_map[] =
{
    SYSHAL_USB_NO_ERROR,
    SYSHAL_USB_ERROR_BUSY,
    SYSHAL_USB_ERROR_FAIL,
};

static USBD_HandleTypeDef hUsbDeviceFS; // USB Device Core handle declaration

static ring_buffer_t rx_buffer;
static uint8_t rx_data[USB_RX_BUF_SIZE];

/**
 * @brief      Initialise the USB instance
 *
 * @return     SYSHAL_USB_NO_ERROR on success
 */
int syshal_usb_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE(); // Enable GPIO pins for USB

    // Setup rx buffer
    rb_init(&rx_buffer, USB_RX_BUF_SIZE, &rx_data[0]);

    USBD_StatusTypeDef retVal = USBD_OK;

    // Init Device Library
    retVal = USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS);
    if (retVal != USBD_OK)
        return hal_error_map[retVal];

    // Add supported class
    retVal = USBD_RegisterClass(&hUsbDeviceFS, USBD_VENDOR_CLASS);
    if (retVal != USBD_OK)
        return hal_error_map[retVal];

    // Start the library
    retVal = USBD_Start(&hUsbDeviceFS);
    if (retVal != USBD_OK)
        return hal_error_map[retVal];

    return hal_error_map[retVal];
}

/**
 * @brief      Deinitialise the USB instance
 *
 * @return     SYSHAL_USB_NO_ERROR on success
 */
int syshal_usb_term(void)
{
    USBD_StatusTypeDef retVal = USBD_DeInit(&hUsbDeviceFS);
    return hal_error_map[retVal];
}

/**
 * @brief      Send the given buffer to our USB host
 *
 * @param[in]  data  The data buffer
 * @param[in]  size  The size of the data buffer in bytes
 *
 * @return     SYSHAL_USB_NO_ERROR on success
 * @return     SYSHAL_USB_ERROR_BUSY if a transfer is already in progress.
 * @return     SYSHAL_USB_ERROR_FAIL if the transfer has failed
 * @return     SYSHAL_USB_ERROR_DISCONNECTED if the USB is disconnected
 */
int syshal_usb_send(uint8_t * data, uint32_t size)
{
    USBD_Vendor_HandleTypeDef_t * hVendor = (USBD_Vendor_HandleTypeDef_t *)hUsbDeviceFS.pClassData;

    if (USBD_STATE_CONFIGURED != hUsbDeviceFS.dev_state)
        return SYSHAL_USB_ERROR_DISCONNECTED;

    if (hVendor->TxPending)
        return SYSHAL_USB_ERROR_BUSY;

    USBD_Vendor_SetTxBuffer(&hUsbDeviceFS, data, size);
    uint8_t result = USBD_Vendor_TransmitPacket(&hUsbDeviceFS);

    return hal_error_map[result];
}

/**
 * @brief      Nonblocking USB receive.
 *
 * @param[out] data  The buffer to store received bytes into
 * @param[in]  size  Maximum number of bytes to receive
 *
 * @return     Actual number of bytes received
 */
int syshal_usb_receive(uint8_t * buffer, uint32_t size)
{
    USBD_Vendor_HandleTypeDef_t * hVendor = (USBD_Vendor_HandleTypeDef_t *)hUsbDeviceFS.pClassData;

    // If we currently are not pending a new transfer
    if (hVendor->RxPending)
        return SYSHAL_USB_ERROR_BUSY;

    if (size < VENDOR_ENDPOINT_PACKET_SIZE)
        return SYSHAL_USB_BUFFER_TOO_SMALL;

    USBD_Vendor_SetRxBuffer(&hUsbDeviceFS, buffer); // Potentially unnecessary?
    USBD_Vendor_ReceivePacket(&hUsbDeviceFS);

    return SYSHAL_USB_NO_ERROR;
}

// Implement MSP hooks that are called by stm32f0xx_hal_pcd
void HAL_PCD_MspInit(PCD_HandleTypeDef * pcdHandle)
{
    if (pcdHandle->Instance == USB)
    {
        /* Peripheral clock enable */
        __HAL_RCC_USB_CLK_ENABLE();

        /* Peripheral interrupt init */
        HAL_NVIC_SetPriority(USB_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USB_IRQn);
    }
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef * pcdHandle)
{
    if (pcdHandle->Instance == USB)
    {
        /* Peripheral clock disable */
        __HAL_RCC_USB_CLK_DISABLE();

        /* Peripheral interrupt Deinit*/
        HAL_NVIC_DisableIRQ(USB_IRQn);
    }
}

/**
 * @brief      This function handles USB global interrupt / USB wake-up
 *             interrupt through EXTI line 18.
 */
void USB_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_FS);
}


/**
 * @brief      This function is called whenever a event occurs on the USB bus.
 *             This should be overriden by config_if.c
 *
 * @param[out] event  The event
 *
 * @return     Return error code
 */
__attribute__((weak)) int syshal_usb_event_handler(syshal_usb_event_t * event)
{
    UNUSED(event);
    DEBUG_PR_WARN("%s Not implemented", __FUNCTION__);

    return SYSHAL_USB_NO_ERROR;
}