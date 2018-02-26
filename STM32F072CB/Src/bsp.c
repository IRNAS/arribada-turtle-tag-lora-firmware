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

#include "bsp.h"

// GPIO definitions
const GPIO_InitTypeDefAndPort_t GPIO_Inits[GPIO_TOTAL_NUMBER] =
{
    {GPIOB, {GPIO_PIN_3, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, 0}}
};

// SPI definitions
const GPIO_InitTypeDefAndInst_t SPI_Inits[SPI_TOTAL_NUMBER] =
{
	{SPI1, {SPI_MODE_MASTER, SPI_DIRECTION_2LINES, SPI_DATASIZE_8BIT, SPI_POLARITY_LOW, SPI_PHASE_1EDGE, SPI_NSS_SOFT, SPI_BAUDRATEPRESCALER_4, SPI_FIRSTBIT_MSB, SPI_TIMODE_DISABLE, SPI_CRCCALCULATION_DISABLE, 7, SPI_CRC_LENGTH_DATASIZE, SPI_NSS_PULSE_ENABLE}},
	{SPI2, {SPI_MODE_MASTER, SPI_DIRECTION_2LINES, SPI_DATASIZE_8BIT, SPI_POLARITY_LOW, SPI_PHASE_1EDGE, SPI_NSS_SOFT, SPI_BAUDRATEPRESCALER_4, SPI_FIRSTBIT_MSB, SPI_TIMODE_DISABLE, SPI_CRCCALCULATION_DISABLE, 7, SPI_CRC_LENGTH_DATASIZE, SPI_NSS_PULSE_ENABLE}}
};