/**
  ******************************************************************************
  * @file     syshal_gpio.c
  * @brief    System hardware abstraction layer for GPIO.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2018 Arribada</center></h2>
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
  *
  ******************************************************************************
  */

#include "syshal_gpio.h"
#include "bsp.h"

/**
 * @brief      Initialise the given GPIO pin
 *
 * @param[in]  pin   The pin
 */
void syshal_gpio_init(uint32_t pin)
{

    if (GPIOA == GPIO_Inits[pin].Port)
        __HAL_RCC_GPIOA_CLK_ENABLE();

    if (GPIOB == GPIO_Inits[pin].Port)
        __HAL_RCC_GPIOB_CLK_ENABLE();

    if (GPIOF == GPIO_Inits[pin].Port)
        __HAL_RCC_GPIOF_CLK_ENABLE();

    HAL_GPIO_Init(GPIO_Inits[pin].Port, &GPIO_Inits[pin].Init);

}

/**
 * @brief      De-initialise the GPIOx peripheral registers to their default reset values
 *
 * @param[in]  pin   The pin
 */
void syshal_gpio_term(uint32_t pin)
{
    HAL_GPIO_DeInit(GPIO_Inits[pin].Port, GPIO_Inits[pin].Init.Pin);
}

/**
 * @brief      Toggle the given GPIO output state
 *
 * @param[in]  pin   The pin
 */
void syshal_gpio_set_output_toggle(uint32_t pin)
{
    HAL_GPIO_TogglePin(GPIO_Inits[pin].Port, GPIO_Inits[pin].Init.Pin);
}

/**
 * @brief      Set the given GPIO pin output to low
 *
 * @param[in]  pin   The pin
 */
void syshal_gpio_set_output_low(uint32_t pin)
{
    HAL_GPIO_WritePin(GPIO_Inits[pin].Port, GPIO_Inits[pin].Init.Pin, GPIO_PIN_RESET);
}

/**
 * @brief      Set the given GPIO pin output to high
 *
 * @param[in]  pin   The pin
 */
void syshal_gpio_set_output_high(uint32_t pin)
{
    HAL_GPIO_WritePin(GPIO_Inits[pin].Port, GPIO_Inits[pin].Init.Pin, GPIO_PIN_SET);
}

/**
 * @brief      Get the given level on the specified GPIO pin
 *
 * @param[in]  pin   The pin
 *
 * @return     true for high, false for low
 */
bool syshal_gpio_get_input(uint32_t pin)
{
    return ( (bool) HAL_GPIO_ReadPin(GPIO_Inits[pin].Port, GPIO_Inits[pin].Init.Pin) );
}