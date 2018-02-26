/* syshal_gpio.h - HAL for GPIO management
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

#ifndef _SYSHAL_GPIO_H_
#define _SYSHAL_GPIO_H_

#include <stdbool.h>
#include "bsp.h"

void syshal_gpio_init(GPIO_Pins_t pin);
void syshal_gpio_term(GPIO_Pins_t pin);
void syshal_gpio_enableInterrupt(GPIO_Pins_t pin, void (*callbackFunc)(void));
void syshal_gpio_disableInterrupt(GPIO_Pins_t pin);
void syshal_gpio_setOutputLow(GPIO_Pins_t pin);
void syshal_gpio_setOutputHigh(GPIO_Pins_t pin);
void syshal_gpio_setOutputToggle(GPIO_Pins_t pin);
bool syshal_gpio_getInput(GPIO_Pins_t pin);

#endif /* _SYSHAL_GPIO_H_ */