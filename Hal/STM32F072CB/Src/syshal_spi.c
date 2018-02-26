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
#include "syshal_spi.h"

// Private variables
static SPI_HandleTypeDef  hspi1;
static SPI_HandleTypeDef  hspi2;

void syshal_spi_init(SPI_t instance)
{

    if (SPI_1 == instance)
    {
        // Populate internal handlers
        hspi1.Instance = SPI_Inits[SPI_1].Instance;
        hspi1.Init = SPI_Inits[SPI_1].Init;

        HAL_SPI_Init(&hspi1);
    }

    if (SPI_2 == instance)
    {
        // Populate internal handlers
        hspi2.Instance = SPI_Inits[SPI_2].Instance;
        hspi2.Init = SPI_Inits[SPI_2].Init;

        HAL_SPI_Init(&hspi2);
    }
}

// Implement MSP hooks that are called by stm32f0xx_hal_spi
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