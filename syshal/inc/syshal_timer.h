/* syshal_timer.h - HAL for MCU timers
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

#ifndef _SYSHAL_TIMER_H_
#define _SYSHAL_TIMER_H_

#include <stdint.h>

#define SYSHAL_TIMER_NO_ERROR          ( 0)
#define SYSHAL_TIMER_INVALID_TIMER_ID  (-1)
#define SYSHAL_TIMER_INVALID_TIME      (-2)

#define SYSHAL_TIMER_NUMBER_OF_TIMERS  (16)

typedef enum
{
    one_shot,  // Only trigger the timer once
    periodic   // Continuely reset the timer everytime it is triggered
} syshal_timer_mode_t;

int syshal_timer_init(void);

int syshal_timer_set(uint32_t timer_id, syshal_timer_mode_t mode, uint32_t seconds);
int syshal_timer_set_ms(uint32_t timer_id, syshal_timer_mode_t mode, uint32_t milliseconds);
int syshal_timer_running(uint32_t timer_id);
int syshal_timer_cancel(uint32_t timer_id);
int syshal_timer_cancel_all(void);

void syshal_timer_tick(void);
void syshal_timer_callback(uint32_t timer_id);

#endif /* _SYSHAL_TIMER_H_ */