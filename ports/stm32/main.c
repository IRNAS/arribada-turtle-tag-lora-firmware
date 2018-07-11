/* Copyright (C) 2018 Arribada
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

// Includes ------------------------------------------------------------------
#include "main.h"
#include "bsp.h"
#include "cexception.h"
#include "debug.h"
#include "sm.h"
#include "sm_main.h"
#include "system_clock.h"
#include "syshal_gpio.h"
#include "syshal_i2c.h"
#include "syshal_spi.h"
#include "syshal_uart.h"
#include "syshal_batt.h"
#include "syshal_gps.h"
#include "syshal_usb.h"
#include "version.h"
#include <string.h>

int main(void)
{
    sm_handle_t state_handle;

    // Reset of all peripherals, Initializes the Flash interface and the Systick
    HAL_Init();

    // Set all pins to Analog to reduce power consumption on unused pins
    GPIO_InitTypeDef GPIO_Init;
    GPIO_Init.Pin = GPIO_PIN_All;
    GPIO_Init.Mode = GPIO_MODE_ANALOG;
    GPIO_Init.Pull = GPIO_NOPULL;
    GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_Init.Alternate = 0;

    // GPIO ports must be clocked to allow changing of settings
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    HAL_GPIO_Init(GPIOA, &GPIO_Init);
    HAL_GPIO_Init(GPIOB, &GPIO_Init);
    HAL_GPIO_Init(GPIOC, &GPIO_Init);
    HAL_GPIO_Init(GPIOD, &GPIO_Init);
    HAL_GPIO_Init(GPIOE, &GPIO_Init);
    HAL_GPIO_Init(GPIOF, &GPIO_Init);

    __HAL_RCC_GPIOA_CLK_DISABLE();
    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_GPIOC_CLK_DISABLE();
    __HAL_RCC_GPIOD_CLK_DISABLE();
    __HAL_RCC_GPIOE_CLK_DISABLE();
    __HAL_RCC_GPIOF_CLK_DISABLE();

    // Configure the system clock
    system_clock_config();

    sm_init(&state_handle, sm_main_states);
    sm_set_next_state(&state_handle, SM_MAIN_BOOT);

    while (1)
    {
        CEXCEPTION_T e = CEXCEPTION_NONE;

        Try
        {
            sm_tick(&state_handle);
        } Catch (e)
        {
            sm_main_exception_handler(e);
        }
    }

    return 0;

}