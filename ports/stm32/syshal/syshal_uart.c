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
#include "ring_buffer.h"
#include "debug.h"

// Private variables
static UART_HandleTypeDef huart1;
static UART_HandleTypeDef huart2;

#define PRINTF_UART_OUTPUT huart2

// Internal variables
static ring_buffer_t rx_buffer[UART_TOTAL_NUMBER];
static uint8_t rx_data[UART_TOTAL_NUMBER][USART_RX_BUF_SIZE];

static inline UART_HandleTypeDef * get_handle(UART_t instance)
{
    if (UART_1 == instance)
        return &huart1;

    if (UART_2 == instance)
        return &huart2;

    return NULL;
}

void syshal_uart_init(UART_t instance)
{
    UART_HandleTypeDef * handle = get_handle(instance);

    // Populate internal handlers
    handle->Instance = UART_Inits[instance].Instance;
    handle->Init = UART_Inits[instance].Init;

    // Setup rx buffer
    rb_init(&rx_buffer[instance], USART_RX_BUF_SIZE, &rx_data[instance][0]);

    // Turn off buffers. This ensure printf prints immediately
    if (handle == &PRINTF_UART_OUTPUT)
    {
        setvbuf(stdin, NULL, _IONBF, 0);
        setvbuf(stdout, NULL, _IONBF, 0);
        setvbuf(stderr, NULL, _IONBF, 0);
    }

    HAL_UART_Init(handle);
}

/**
 * @brief Read one character from a serial port.
 *
 * It's not safe to call this function if the serial port has no data
 * available.
 *
 * @param dev Serial port to read from
 * @return byte read
 * @see usart_data_available()
 */
static inline uint8_t usart_getc(UART_t instance)
{
    return rb_remove(&rx_buffer[instance]); // Remove and return the first item from a ring buffer.
}

/**
 * @brief      Return the amount of data available in a serial port's RX buffer.
 *
 * @param[in]  instance  The serial port to check
 *
 * @return     Number of bytes in serial port's RX buffer.
 */
uint32_t syshal_uart_available(UART_t instance)
{
    return rb_full_count(&rx_buffer[instance]); // Return the number of elements stored in the ring buffer
}

/**
 * @brief Nonblocking USART receive.
 * @param dev Serial port to receive bytes from
 * @param buf Buffer to store received bytes into
 * @param size Maximum number of bytes to store
 * @return Number of bytes received
 */
uint32_t syshal_uart_receive(UART_t instance, uint8_t * data, uint32_t size)
{
    uint32_t count = rb_full_count(&rx_buffer[instance]);

    if (size > count)
        size = count;

    for (uint32_t i = 0; i < size; ++i)
    {
        *data++ = usart_getc(instance);
    }

    return size;
}

void syshal_uart_transfer(UART_t instance, uint8_t * data, uint32_t size)
{

    if (!size)
        return;

    UART_HandleTypeDef * handle = get_handle(instance);
    HAL_StatusTypeDef status;

    // Wait for UART to be free
    while (HAL_UART_GetState(handle) != HAL_UART_STATE_READY)
    {}
    status = HAL_UART_Transmit(handle, data, size, UART_TIMEOUT);

    if (HAL_OK != status)
    {
        DEBUG_PR_ERROR("%s failed with %d", __FUNCTION__, status);
    }
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
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); // Enable the UART Data Register not empty Interrupt

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
        __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE); // Enable the UART Data Register not empty Interrupt

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

static inline void uart_irq(UART_t instance)
{
    UART_HandleTypeDef * huart = get_handle(instance);

    // Did we receive data ?
    if (__HAL_UART_GET_IT(huart, UART_IT_RXNE))
    {
        uint8_t rxBuffer; // Read one byte from the receive data register

        rxBuffer = huart->Instance->RDR & 0xFF;

#ifdef UART_SAFE_INSERT
        // If the buffer is full and the user defines UART_SAFE_INSERT, ignore new bytes.
        rb_safe_insert(&rx_buffer[instance], rxBuffer);
#else
        // If the buffer is full overwrite data
        rb_push_insert(&rx_buffer[instance], rxBuffer);
#endif

    }

}

/**
* @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
*/
void USART1_IRQHandler(void)
{
    uart_irq(UART_1);
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
    uart_irq(UART_2);
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
    {}

    HAL_StatusTypeDef status = HAL_UART_Transmit(&PRINTF_UART_OUTPUT, (uint8_t *)data, len, UART_TIMEOUT);

    // return # of bytes written - as best we can tell
    return (status == HAL_OK ? len : 0);
}