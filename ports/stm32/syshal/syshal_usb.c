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

#include "syshal_usb.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_vendor.h"

static USBD_HandleTypeDef hUsbDeviceFS; // USB Device Core handle declaration
extern PCD_HandleTypeDef hpcd_USB_FS; // Pulled in from usbd_conf.c FIXME: STM32cubeMx generated not using extern passed via header

int syshal_usb_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE(); // Enable GPIO pins for USB

    // Init Device Library, add supported class and start the library
    USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS);
    USBD_RegisterClass(&hUsbDeviceFS, USBD_VENDOR_CLASS);
    USBD_Start(&hUsbDeviceFS);

    return 0;
}

int syshal_usb_transfer(uint8_t * data, uint32_t size)
{
    USBD_Vendor_HandleTypeDef_t * hVendor = (USBD_Vendor_HandleTypeDef_t *)hUsbDeviceFS.pClassData;

    if (hVendor->TxState != 0)
        return USBD_BUSY;

    USBD_Vendor_SetTxBuffer(&hUsbDeviceFS, data, size);
    uint8_t result = USBD_Vendor_TransmitPacket(&hUsbDeviceFS);

    return result;
}

int syshal_usb_receive(uint8_t * data, uint32_t size)
{
    UNUSED(size); // NOTE: Is this correct?
    USBD_Vendor_SetRxBuffer(&hUsbDeviceFS, &data[0]);
    USBD_Vendor_ReceivePacket(&hUsbDeviceFS);
    return USBD_OK;
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