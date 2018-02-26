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

#ifndef _SYSHAL_SPI_H_
#define _SYSHAL_SPI_H_

/*
typedef struct
{
    uint32_t mode; // Specifies the SPI operating mode
    uint32_t direction; // Specifies the SPI bidirectional mode state
    uint32_t dataSize; // Specifies the SPI data size
    uint32_t CLKPolarity; // Specifies the serial clock steady state
    uint32_t CLKPhase; // Specifies the clock active edge for the bit capture
    uint32_t baudRate;  // Specifies the Baud Rate
    uint32_t firstBit; // Specifies whether data transfers start from MSB or LSB bit
} stm32_spi_init;

typedef struct
{
    SPI_TypeDef * instance; // SPI registers base address
    stm32_spi_init init; // SPI communication parameters
    uint32_t errorCode; // SPI Error code
} stm32_spi_handle;
*/

void syshal_spi_init(SPI_t instance); // Returns a handle to the device initilaised
void syshal_spi_term(SPI_t instance);
void syshal_spi_transfer(SPI_t instance, uint8_t * data, uint32_t length);
uint32_t syshal_spi_receive(SPI_t instance, uint8_t * data); // returns length of data read

#endif /* _SYSHAL_SPI_H_ */