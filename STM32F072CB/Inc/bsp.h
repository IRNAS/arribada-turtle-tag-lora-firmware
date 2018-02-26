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

#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA

// GPIO definitions
typedef enum
{
    GPIO_LED3,
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
} GPIO_InitTypeDefAndInst_t;

extern const GPIO_InitTypeDefAndInst_t SPI_Inits[SPI_TOTAL_NUMBER];

// I2C definitions

// UART definitions

#endif /* _BSP_H_ */