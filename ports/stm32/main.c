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
#include "syshal_gpio.h"
#include "syshal_i2c.h"
#include "syshal_spi.h"
#include "syshal_uart.h"
#include "syshal_batt.h"
#include "syshal_gps.h"
#include "version.h"
#include <string.h>

// Private variables ---------------------------------------------------------

// Private function prototypes -----------------------------------------------
void SystemClock_Config(void);

// Callback function overrides -----------------------------------------------
void syshal_gps_callback(syshal_gps_event_t event)
{
    switch (event)
    {
        case SYSHAL_GPS_EVENT_LOCK_MADE:
            DEBUG_PR_SYS("GPS lock made");
            break;
        case SYSHAL_GPS_EVENT_LOCK_LOST:
            DEBUG_PR_SYS("GPS lock lost");
            break;
        case SYSHAL_GPS_EVENT_NEW_DATA:
            __NOP(); // Labels can only be followed by statements
            syshal_gpio_set_output_toggle(GPIO_LED3);
            uint32_t iTOW;
            int32_t longitude, latitude, height;
            syshal_gps_get_location(&iTOW, &longitude, &latitude, &height);
            DEBUG_PR_INFO("New Location - time: %u ms, Long: %dE-7 deg, Lat: %dE-7 deg, Height: %d mm", iTOW, longitude, latitude, height);
            break;
    }
}

int main(void)
{

    // Reset of all peripherals, Initializes the Flash interface and the Systick
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();

    // Initialize all configured peripherals
    syshal_gpio_init(GPIO_LED3);
    syshal_uart_init(UART_1);
    syshal_uart_init(UART_2);
    //syshal_spi_init(SPI_1);
    //syshal_i2c_init(I2C_1);
    //syshal_batt_init(I2C_1);
    syshal_gps_init();

    // Print General System Info
    DEBUG_PR_SYS("Arribada Tracker Device");
    DEBUG_PR_SYS("Version:  %s", GIT_VERSION);
    DEBUG_PR_SYS("Compiled: %s %s With %s", COMPILE_DATE, COMPILE_TIME, COMPILER_NAME);

    uint32_t deltaTime = syshal_time_get_ticks_ms();
    bool gpsAwake = true;

    while (1)
    {

        uint32_t timeElapsed = syshal_time_get_ticks_ms() - deltaTime;

        if (timeElapsed > 5000)
        {
            deltaTime += 5000;
            if (gpsAwake)
            {
                syshal_gps_shutdown();
                gpsAwake = false;
            }
            else
            {
                syshal_gps_wake_up();
                gpsAwake = true;
            }
        }

        syshal_gps_tick();

    }

}

void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    // Initializes the CPU, AHB and APB busses clocks
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Initializes the CPU, AHB and APB busses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

// @brief  This function is executed in case of error occurrence
void _Error_Handler(char * file, int line)
{
    // User can add his own implementation to report the HAL error return state
    while (1)
    {
        // Toggle LED3 fast for error
        syshal_gpio_set_output_toggle(GPIO_LED3);
        syshal_time_delay_ms(100);
    }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t * file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif