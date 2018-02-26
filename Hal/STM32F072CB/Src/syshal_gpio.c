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