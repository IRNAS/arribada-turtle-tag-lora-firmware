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
#include "debug.h"
#include "sm.h"
#include "syshal_gpio.h"
#include "syshal_i2c.h"
#include "syshal_spi.h"
#include "syshal_uart.h"
#include "syshal_batt.h"
#include "syshal_gps.h"
#include "syshal_usb.h"
#include "version.h"
#include <string.h>

// Private variables ---------------------------------------------------------

// Private function prototypes -----------------------------------------------
void SystemClock_Config(void);

int main(void)
{

    // Reset of all peripherals, Initializes the Flash interface and the Systick
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();

    while (1)
    {
        sm_iterate();
    }

    return 0;

}

/**
 * @brief      Set up all internal clock prescalers and routing. Does not
 *             include clock gating
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    // Configure LSE Drive Capability
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_HIGH);

    // Initializes the CPU, AHB and APB busses clocks
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE
                                       | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Initializes the CPU, AHB and APB busses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_USART1
                                         | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_I2C1
                                         | RCC_PERIPHCLK_RTC;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Configure the Systick interrupt time
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    // Configure the Systick
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    // SysTick_IRQn interrupt configuration
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

// @brief      This function is executed in case of error occurrence
//
// @param[in]  file  The file the error occured in
// @param[in]  line  The line the error occured at
//
void _Error_Handler(char * file, int line)
{
    // User can add his own implementation to report the HAL error return state
    while (1)
    {
    }
}