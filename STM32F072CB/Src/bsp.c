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

#include "bsp.h"

// GPIO definitions
const GPIO_InitTypeDefAndPort_t GPIO_Inits[GPIO_TOTAL_NUMBER] =
{
    {GPIOB, {GPIO_PIN_3,  GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, 0}},               // GPIO_LED3
    {GPIOA, {GPIO_PIN_2,  GPIO_MODE_AF_PP,     GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF1_USART2}}, // GPIO_VCP_TX
    {GPIOA, {GPIO_PIN_15, GPIO_MODE_AF_PP,     GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF1_USART2}}  // GPIO_VCP_RX
};

// SPI definitions
const SPI_InitTypeDefAndInst_t SPI_Inits[SPI_TOTAL_NUMBER] =
{
    {SPI1, {SPI_MODE_MASTER, SPI_DIRECTION_2LINES, SPI_DATASIZE_8BIT, SPI_POLARITY_LOW, SPI_PHASE_1EDGE, SPI_NSS_SOFT, SPI_BAUDRATEPRESCALER_4, SPI_FIRSTBIT_MSB, SPI_TIMODE_DISABLE, SPI_CRCCALCULATION_DISABLE, 7, SPI_CRC_LENGTH_DATASIZE, SPI_NSS_PULSE_ENABLE}},
    {SPI2, {SPI_MODE_MASTER, SPI_DIRECTION_2LINES, SPI_DATASIZE_8BIT, SPI_POLARITY_LOW, SPI_PHASE_1EDGE, SPI_NSS_SOFT, SPI_BAUDRATEPRESCALER_4, SPI_FIRSTBIT_MSB, SPI_TIMODE_DISABLE, SPI_CRCCALCULATION_DISABLE, 7, SPI_CRC_LENGTH_DATASIZE, SPI_NSS_PULSE_ENABLE}}
};

// UART definitions
const UART_InitTypeDefAndInst_t UART_Inits[SPI_TOTAL_NUMBER] =
{
    // Instance,BaudRate,WordLength,StopBits,Parity,Mode,HwFlowCtl,OverSampling,OneBitSampling
    {USART1, {115200, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE, UART_MODE_TX_RX, UART_HWCONTROL_NONE, 0, 0}},
    {USART2, {115200, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE, UART_MODE_TX_RX, UART_HWCONTROL_NONE, 0, 0}}
};