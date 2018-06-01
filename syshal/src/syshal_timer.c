/* syshal_timer.c - HAL for MCU timers
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

#include "syshal_timer.h"
#include "syshal_rtc.h"
#include "debug.h"

#define SECONDS_IN_A_DAY (24 * 60 * 60)

static int32_t syshal_timer[SYSHAL_TIMER_NUMBER_OF_TIMERS];

uint32_t syshal_timer_time_in_seconds_priv(void)
{
    // Get the current time of day in seconds
    syshal_rtc_data_and_time_t current_date_time;
    syshal_rtc_get_date_and_time(&current_date_time);

    return (uint32_t) (current_date_time.seconds + (current_date_time.minutes * 60) + (current_date_time.hours * 60 * 60)) % SECONDS_IN_A_DAY;
}

int syshal_timer_init(void)
{
    return syshal_timer_cancel_all();
}

int syshal_timer_set(uint32_t timer_id, uint32_t seconds)
{
    if (timer_id > SYSHAL_TIMER_NUMBER_OF_TIMERS)
        return SYSHAL_TIMER_INVALID_TIMER_ID;

    if (seconds >= SECONDS_IN_A_DAY)
        return SYSHAL_TIMER_INVALID_TIME;

    syshal_timer[timer_id] = (syshal_timer_time_in_seconds_priv() + seconds) % SECONDS_IN_A_DAY;

    return SYSHAL_TIMER_NO_ERROR;
}

int syshal_timer_cancel(uint32_t timer_id)
{
    if (timer_id > SYSHAL_TIMER_NUMBER_OF_TIMERS)
        return SYSHAL_TIMER_INVALID_TIMER_ID;

    syshal_timer[timer_id] = SYSHAL_TIMER_NOT_SET;

    return SYSHAL_TIMER_NO_ERROR;
}

int syshal_timer_cancel_all(void)
{
    for (uint32_t i = 0; i < SYSHAL_TIMER_NUMBER_OF_TIMERS; ++i)
        syshal_timer_cancel(i);

    return SYSHAL_TIMER_NO_ERROR;
}

void syshal_timer_tick(void)
{
    uint32_t current_time_seconds = syshal_timer_time_in_seconds_priv();

    for (uint32_t i = 0; i < SYSHAL_TIMER_NUMBER_OF_TIMERS; ++i)
    {
        if (syshal_timer[i] <= current_time_seconds)
        {
            syshal_timer_cancel(i);
            syshal_timer_callback(i);
        }
    }
}

__attribute__((weak)) void syshal_timer_callback(uint32_t timer_id)
{
    DEBUG_PR_WARN("%s() Not implemented, id: %lu", __FUNCTION__, timer_id);
}