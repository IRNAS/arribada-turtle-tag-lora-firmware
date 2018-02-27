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

#include "stm32f0xx_hal.h"
#include "syshal_gpio.h"
#include "syshal_spi.h"
#include "debug.h"

// Private variables
static SPI_HandleTypeDef hspi1;
static SPI_HandleTypeDef hspi2;

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

void syshal_spi_transfer(SPI_t instance, uint8_t * data, uint32_t size)
{
    HAL_StatusTypeDef status;

    if (SPI_1 == instance)
        status = HAL_SPI_Transmit(&hspi1, data, size, SPI_TIMEOUT);

    if (SPI_2 == instance)
        status = HAL_SPI_Transmit(&hspi2, data, size, SPI_TIMEOUT);

    if (HAL_OK != status)
    {
        DEBUG_PR_ERROR("%s failed with %d", __FUNCTION__, status);
    }
}

uint32_t syshal_spi_receive(SPI_t instance, uint8_t * data, uint32_t size)
{
    HAL_StatusTypeDef status;

    if (SPI_1 == instance)
        status = HAL_SPI_Receive(&hspi1, data, size, SPI_TIMEOUT);

    if (SPI_2 == instance)
        status = HAL_SPI_Receive(&hspi2, data, size, SPI_TIMEOUT);

    // return number of bytes read as best we can tell
    if (HAL_OK != status)
    {
        DEBUG_PR_ERROR("%s failed with %d", __FUNCTION__, status);
        return 0;
    }

    return size;
}

// Implement MSP hooks that are called by stm32f0xx_hal_spi
void HAL_SPI_MspInit(SPI_HandleTypeDef * hspi)
{

    if (hspi->Instance == SPI1)
    {
        // Peripheral clock enable
        __HAL_RCC_SPI1_CLK_ENABLE();

        // SPI1 GPIO Configuration
        syshal_gpio_init(GPIO_SPI1_MOSI);
        syshal_gpio_init(GPIO_SPI1_MISO);
        syshal_gpio_init(GPIO_SPI1_SCK);
    }

    if (hspi->Instance == SPI2)
    {
        // Peripheral clock enable
        __HAL_RCC_SPI2_CLK_ENABLE();

        // SPI2 GPIO Configuration
        // syshal_gpio_init(GPIO_SPI2_MOSI); // TODO needs implementing
        // syshal_gpio_init(GPIO_SPI2_MISO); // TODO needs implementing
        // syshal_gpio_init(GPIO_SPI2_SCK);  // TODO needs implementing
    }

}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef * hspi)
{

    if (hspi->Instance == SPI1)
    {
        // Peripheral clock disable
        __HAL_RCC_SPI1_CLK_DISABLE();

        // SPI1 GPIO Configuration
        syshal_gpio_term(GPIO_SPI1_MOSI);
        syshal_gpio_term(GPIO_SPI1_MISO);
        syshal_gpio_term(GPIO_SPI1_SCK);
    }

    if (hspi->Instance == SPI2)
    {
        // Peripheral clock disable
        __HAL_RCC_SPI2_CLK_DISABLE();

        // SPI2 GPIO Configuration
        // syshal_gpio_term(GPIO_SPI2_MOSI); // TODO needs implementing
        // syshal_gpio_term(GPIO_SPI2_MISO); // TODO needs implementing
        // syshal_gpio_term(GPIO_SPI2_SCK);  // TODO needs implementing
    }

}