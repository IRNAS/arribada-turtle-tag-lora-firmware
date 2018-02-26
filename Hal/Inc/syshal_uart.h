/* syshal_uart.h - HAL for UART
 *
 * Copyright (C) 2018 Arribada
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

#ifndef _SYSHAL_UART_H_
#define _SYSHAL_UART_H_

#include "bsp.h"

void syshal_uart_init(UART_t instance);
void syshal_uart_term(UART_t instance);
void syshal_uart_transfer(UART_t instance, uint8_t * data, uint32_t length);
uint32_t syshal_uart_receive(UART_t instance, uint8_t * data, uint32_t length); // returns length of data read

#endif /* _SYSHAL_UART_H_ */