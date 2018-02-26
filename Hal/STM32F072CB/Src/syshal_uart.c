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

#include "syshal_uart.h"

// Private variables
static UART_HandleTypeDef huart1;
static UART_HandleTypeDef huart2;

void syshal_uart_init(UART_t instance)
{

    if (UART_1 == instance)
    {
        // Populate internal handlers
        huart1.Instance = UART_Inits[UART_1].Instance;
        huart1.Init = UART_Inits[UART_1].Init;

        HAL_UART_Init(&huart1);
    }

    if (UART_2 == instance)
    {
        // Populate internal handlers
        huart2.Instance = UART_Inits[UART_2].Instance;
        huart2.Init = UART_Inits[UART_2].Init;

        HAL_UART_Init(&huart2);
    }
}

void syshal_uart_transfer(UART_t instance, uint8_t * data, uint32_t length)
{
    if (UART_1 == instance)
    {
        HAL_UART_Transmit(&huart1, data, length, 0xFFFF); // 0xFFFF timeout
    }

    if (UART_2 == instance)
    {
        HAL_UART_Transmit(&huart2, data, length, 0xFFFF);
    }
}