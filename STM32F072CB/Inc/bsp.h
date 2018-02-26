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

#ifndef _BSP_H_
#define _BSP_H_

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_hal_spi.h"
#include "stm32f0xx_hal_uart.h"

#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA

// GPIO definitions
typedef enum
{
    GPIO_LED3,
    GPIO_VCP_TX,
    GPIO_VCP_RX,
    GPIO_TOTAL_NUMBER,
} GPIO_Pins_t;

// Merge port number and init structure
typedef struct
{
    GPIO_TypeDef * Port;
    GPIO_InitTypeDef Init;
} GPIO_InitTypeDefAndPort_t;

// Pin definitions
extern const GPIO_InitTypeDefAndPort_t GPIO_Inits[GPIO_TOTAL_NUMBER];

// SPI definitions
#define SPI_TIMEOUT 1000

typedef enum
{
    SPI_1,
    SPI_2,
    SPI_TOTAL_NUMBER,
} SPI_t;

typedef struct
{
    SPI_TypeDef * Instance;
    SPI_InitTypeDef Init;
} SPI_InitTypeDefAndInst_t;

extern const SPI_InitTypeDefAndInst_t SPI_Inits[SPI_TOTAL_NUMBER];

// I2C definitions

// UART definitions
#define UART_TIMEOUT 1000

typedef enum
{
    UART_1,
    UART_2,
    UART_TOTAL_NUMBER,
} UART_t;

typedef struct
{
    USART_TypeDef * Instance;
    UART_InitTypeDef Init;
} UART_InitTypeDefAndInst_t;

extern const UART_InitTypeDefAndInst_t UART_Inits[UART_TOTAL_NUMBER];

#endif /* _BSP_H_ */