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

#include <errno.h>
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
#include "stm32f0xx_hal.h"
#include "syshal_gpio.h"
#include "syshal_uart.h"
#include "debug.h"

// Private variables
static UART_HandleTypeDef huart1;
static UART_HandleTypeDef huart2;

#define PRINTF_UART_OUTPUT huart2

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

    // Turn off buffers. This ensure printf prints immediately
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
}

void syshal_uart_transfer(UART_t instance, uint8_t * data, uint32_t size)
{
    HAL_StatusTypeDef status;

    if (UART_1 == instance)
    {
        // Wait for UART to be free
        while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
        {

        }
        status = HAL_UART_Transmit_IT(&huart1, data, size);
    }

    if (UART_2 == instance)
    {
        // Wait for UART to be free
        while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
        {

        }

        status = HAL_UART_Transmit_IT(&huart2, data, size);
    }

    if (HAL_OK != status)
    {
        DEBUG_PR_ERROR("%s failed with %d", __FUNCTION__, status);
    }
}

uint32_t syshal_uart_receive(UART_t instance, uint8_t * data, uint32_t size)
{
    HAL_StatusTypeDef status;

    if (UART_1 == instance)
        status = HAL_UART_Receive_IT(&huart1, data, size);

    if (UART_2 == instance)
        status = HAL_UART_Receive_IT(&huart2, data, size);

    if (HAL_OK != status)
    {
        DEBUG_PR_ERROR("%s failed with %d", __FUNCTION__, status);
        return 0;
    }

    return size;
}

// Implement MSP hooks that are called by stm32f0xx_hal_uart
void HAL_UART_MspInit(UART_HandleTypeDef * huart)
{

    if (huart->Instance == USART1)
    {
        // Peripheral clock disable
        __HAL_RCC_USART1_CLK_ENABLE();

        // USART1 GPIO Configuration
        syshal_gpio_init(GPIO_UART1_TX);
        syshal_gpio_init(GPIO_UART1_RX);

        // USART1 interrupt Init
        HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }

    if (huart->Instance == USART2)
    {
        // Peripheral clock enable
        __HAL_RCC_USART2_CLK_ENABLE();

        // USART2 GPIO Configuration
        syshal_gpio_init(GPIO_UART2_TX);
        syshal_gpio_init(GPIO_UART2_RX);

        // USART2 interrupt Init
        HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }

}

void HAL_UART_MspDeInit(UART_HandleTypeDef * huart)
{

    if (huart->Instance == USART1)
    {
        // Peripheral clock disable
        __HAL_RCC_USART1_CLK_DISABLE();

        // USART1 GPIO Configuration
        syshal_gpio_term(GPIO_UART1_TX);
        syshal_gpio_term(GPIO_UART1_RX);

        // USART1 interrupt DeInit
        HAL_NVIC_DisableIRQ(USART1_IRQn);
    }

    if (huart->Instance == USART2)
    {
        // Peripheral clock disable
        __HAL_RCC_USART2_CLK_DISABLE();

        // USART2 GPIO Configuration
        syshal_gpio_term(GPIO_UART2_TX);
        syshal_gpio_term(GPIO_UART2_RX);

        // USART2 interrupt DeInit
        HAL_NVIC_DisableIRQ(USART2_IRQn);
    }

}

/**
* @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
*/
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

// Override _write function to enable printf use
int _write(int file, char * data, int len)
{
    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
    {
        errno = EBADF;
        return -1;
    }

    // Wait for UART to be free
    while (HAL_UART_GetState(&PRINTF_UART_OUTPUT) != HAL_UART_STATE_READY)
    {

    }

    HAL_StatusTypeDef status = HAL_UART_Transmit_IT(&PRINTF_UART_OUTPUT, (uint8_t *)data, len);

    // return # of bytes written - as best we can tell
    return (status == HAL_OK ? len : 0);
}