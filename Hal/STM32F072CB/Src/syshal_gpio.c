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

#include "syshal_gpio.h"

void syshal_gpio_init(GPIO_Pins_t pin)
{

    if (GPIOA == GPIO_Inits[pin].Port)
        __HAL_RCC_GPIOA_CLK_ENABLE();

    if (GPIOB == GPIO_Inits[pin].Port)
        __HAL_RCC_GPIOB_CLK_ENABLE();

    if (GPIOF == GPIO_Inits[pin].Port)
        __HAL_RCC_GPIOF_CLK_ENABLE();

    HAL_GPIO_Init(GPIO_Inits[pin].Port, &GPIO_Inits[pin].Init);

}

// De-initialise the GPIOx peripheral registers to their default reset values
void syshal_gpio_term(GPIO_Pins_t pin)
{
    HAL_GPIO_DeInit(GPIO_Inits[pin].Port, &GPIO_Inits[pin].Init);
}

void syshal_gpio_setOutputToggle(GPIO_Pins_t pin)
{
    HAL_GPIO_TogglePin(GPIO_Inits[pin].Port, GPIO_Inits[pin].Init.Pin);
}

void syshal_gpio_setOutputLow(GPIO_Pins_t pin)
{
    HAL_GPIO_WritePin(GPIO_Inits[pin].Port, GPIO_Inits[pin].Init.Pin, GPIO_PIN_RESET);
}

void syshal_gpio_setOutputHigh(GPIO_Pins_t pin)
{
    HAL_GPIO_WritePin(GPIO_Inits[pin].Port, GPIO_Inits[pin].Init.Pin, GPIO_PIN_SET);
}

bool syshal_gpio_getInput(GPIO_Pins_t pin)
{
    return ( (bool) HAL_GPIO_ReadPin(GPIO_Inits[pin].Port, GPIO_Inits[pin].Init.Pin) );
}