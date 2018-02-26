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

#ifndef _BSP_H_
#define _BSP_H_

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_hal_spi.h"
#include "stm32f0xx_hal_uart.h"
#include "stm32f0xx_hal_i2c.h"

#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA

///////////////////////////////// GPIO definitions ////////////////////////////////
typedef enum
{
    GPIO_LED3,
    GPIO_UART2_TX,
    GPIO_UART2_RX,
    GPIO_I2C1_SCL,
    GPIO_I2C1_SDA,
    GPIO_SPI1_SCK,
    GPIO_SPI1_MISO,
    GPIO_SPI1_MOSI,
    GPIO_TOTAL_NUMBER
} GPIO_Pins_t;

// Merge port number and init structure
typedef struct
{
    GPIO_TypeDef * Port;
    GPIO_InitTypeDef Init;
} GPIO_InitTypeDefAndPort_t;

extern const GPIO_InitTypeDefAndPort_t GPIO_Inits[GPIO_TOTAL_NUMBER];

///////////////////////////////// SPI definitions /////////////////////////////////
#define SPI_TIMEOUT 1000

typedef enum
{
    SPI_1,
    SPI_2,
    SPI_TOTAL_NUMBER
} SPI_t;

typedef struct
{
    SPI_TypeDef * Instance;
    SPI_InitTypeDef Init;
} SPI_InitTypeDefAndInst_t;

extern const SPI_InitTypeDefAndInst_t SPI_Inits[SPI_TOTAL_NUMBER];

///////////////////////////////// I2C definitions /////////////////////////////////
#define I2C_TIMEOUT 1000

typedef enum
{
    I2C_1,
//    I2C_2,
    I2C_TOTAL_NUMBER
} I2C_t;

typedef struct
{
    I2C_TypeDef * Instance;
    I2C_InitTypeDef Init;
} I2C_InitTypeDefAndInst_t;

extern const I2C_InitTypeDefAndInst_t I2C_Inits[I2C_TOTAL_NUMBER];

///////////////////////////////// UART definitions ////////////////////////////////
#define UART_TIMEOUT 1000

typedef enum
{
    UART_1,
    UART_2,
    UART_TOTAL_NUMBER
} UART_t;

typedef struct
{
    USART_TypeDef * Instance;
    UART_InitTypeDef Init;
} UART_InitTypeDefAndInst_t;

extern const UART_InitTypeDefAndInst_t UART_Inits[UART_TOTAL_NUMBER];

#endif /* _BSP_H_ */