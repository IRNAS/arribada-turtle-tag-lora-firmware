/* Copyright 2018 Arribada
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "stm32f0xx_hal.h"
#include "syshal_gpio.h"
#include "bsp.h"

// Initializes the Global MSP.
void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    // System interrupt ini
    // SVC_IRQn interrupt configuration
    HAL_NVIC_SetPriority(SVC_IRQn, 0, 0);
    // PendSV_IRQn interrupt configuration
    HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
    // SysTick_IRQn interrupt configuration
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void HAL_SPI_MspInit(SPI_HandleTypeDef * hspi)
{

    GPIO_InitTypeDef GPIO_InitStruct;
    if (hspi->Instance == SPI1)
    {
        // Peripheral clock enable
        __HAL_RCC_SPI1_CLK_ENABLE();

        // SPI1 GPIO Configuration
        GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }

}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef * hspi)
{

    if (hspi->Instance == SPI1)
    {
        // Peripheral clock disable
        __HAL_RCC_SPI1_CLK_DISABLE();

        // SPI1 GPIO Configuration
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    }

}

void HAL_UART_MspInit(UART_HandleTypeDef * huart)
{

    if (huart->Instance == USART2)
    {
        // Peripheral clock enable
        __HAL_RCC_USART2_CLK_ENABLE();

        // USART2 GPIO Configuration
        syshal_gpio_init(GPIO_VCP_TX);
        syshal_gpio_init(GPIO_VCP_RX);
    }

}

void HAL_UART_MspDeInit(UART_HandleTypeDef * huart)
{

    if (huart->Instance == USART2)
    {
        // Peripheral clock disable
        __HAL_RCC_USART2_CLK_DISABLE();

        // USART2 GPIO Configuration
        syshal_gpio_init(GPIO_VCP_TX);
        syshal_gpio_init(GPIO_VCP_RX);
    }

}