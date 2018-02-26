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

#ifndef _SYSHAL_GPIO_H_
#define _SYSHAL_GPIO_H_

#include <stdbool.h>
#include "bsp.h"

/*
typedef struct ( uint32_t pin_number,
                 uint32_t pin_port
               ) stm32_gpio_pin;

typedef struct ( stm32_gpio_pin_dir_t      dir,
                 stm32_gpio_pin_input_t    input,
                 stm32_gpio_pin_pull_t     pull,
                 stm32_gpio_pin_drive_t    drive,
                 stm32_gpio_pin_sense_t    sense
               ) stm32_gpio_settings;
*/

void syshal_gpio_init(GPIO_Pins_t pin);
void syshal_gpio_term(void);
void syshal_gpio_enableInterrupt(GPIO_Pins_t pin, void (*callbackFunc)(void));
void syshal_gpio_disableInterrupt(GPIO_Pins_t pin);
void syshal_gpio_setOutputLow(GPIO_Pins_t pin);
void syshal_gpio_setOutputHigh(GPIO_Pins_t pin);
void syshal_gpio_setOutputToggle(GPIO_Pins_t pin);
bool syshal_gpio_getInput(GPIO_Pins_t pin);

#endif /* _SYSHAL_GPIO_H_ */