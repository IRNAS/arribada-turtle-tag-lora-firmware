/* syshal_rtc.c - HAL for RTC device
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

#include "stm32f0xx_hal.h"
#include "syshal_rtc.h"
#include "sys_config.h"
#include "debug.h"

#define YEAR_OFFSET (2018) // RTC can only handle 100 years so offset by 2000

RTC_HandleTypeDef rtc_handle;

// HAL to SYSHAL error code mapping table
static int hal_error_map[] =
{
    SYSHAL_RTC_NO_ERROR,
    SYSHAL_RTC_ERROR_DEVICE,
    SYSHAL_RTC_ERROR_BUSY,
    SYSHAL_RTC_ERROR_TIMEOUT,
};

int syshal_rtc_init(void)
{
    HAL_StatusTypeDef status;

    rtc_handle.Instance = RTC;
    rtc_handle.Init.HourFormat = RTC_HOURFORMAT_24;
    rtc_handle.Init.AsynchPrediv = 127;
    rtc_handle.Init.SynchPrediv = 255;
    rtc_handle.Init.OutPut = RTC_OUTPUT_DISABLE;
    rtc_handle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    rtc_handle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

    status = HAL_RTC_Init(&rtc_handle);

    sys_config.sys_config_rtc_current_date_and_time.hdr.set = true;

    return hal_error_map[status];
}

int syshal_rtc_set_date_and_time(syshal_rtc_data_and_time_t date_time)
{
    HAL_StatusTypeDef status;

    RTC_DateTypeDef date;
    date.WeekDay = RTC_WEEKDAY_MONDAY; // Unused as we don't track the name of the day
    date.Year = date_time.year - YEAR_OFFSET;
    date.Month = date_time.month;
    date.Date = date_time.day;

    status = HAL_RTC_SetDate(&rtc_handle, &date, RTC_FORMAT_BIN);
    if (HAL_OK != status)
        return hal_error_map[status];

    RTC_TimeTypeDef time;
    time.Hours = date_time.hours;
    time.Minutes = date_time.minutes;
    time.Seconds = date_time.seconds;
    time.SubSeconds = 0;
    time.SecondFraction = 0;
    time.TimeFormat = RTC_HOURFORMAT12_AM;
    time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    time.StoreOperation = RTC_STOREOPERATION_SET;

    status = HAL_RTC_SetTime(&rtc_handle, &time, RTC_FORMAT_BIN);

    return hal_error_map[status];
}

int syshal_rtc_get_date_and_time(syshal_rtc_data_and_time_t * date_time)
{
    HAL_StatusTypeDef status;

    // You must call HAL_RTC_GetDate() after HAL_RTC_GetTime() to unlock the values in
    // the higher-order calendar shadow registers to ensure consistency between the time
    // and date values. Reading RTC current time locks the values in calendar shadow
    // registers until Current date is read
    RTC_TimeTypeDef time;
    status = HAL_RTC_GetTime(&rtc_handle, &time, RTC_FORMAT_BIN);
    date_time->hours = time.Hours;
    date_time->minutes = time.Minutes;
    date_time->seconds = time.Seconds;

    RTC_DateTypeDef date;
    status = HAL_RTC_GetDate(&rtc_handle, &date, RTC_FORMAT_BIN);
    if (HAL_OK != status)
        return hal_error_map[status];

    date_time->year = date.Year + YEAR_OFFSET;
    date_time->month = date.Month;
    date_time->day = date.Date;

    return hal_error_map[status];
}

int syshal_rtc_timer(uint32_t seconds)
{
    if (seconds >= (60 * 60 * 24))
        return SYSHAL_RTC_INVALID_PARAMETER; // We can't set a timer for greater than 24 hours

    RTC_AlarmTypeDef alarm_handle;

    // Get the current time
    syshal_rtc_data_and_time_t current_date_time;
    syshal_rtc_get_date_and_time(&current_date_time);

    // Add the number of seconds to our current date_time
    uint32_t calc_hours, calc_minutes, calc_seconds;

    calc_seconds = current_date_time.seconds + seconds;
    calc_minutes = current_date_time.minutes + (calc_seconds / 60);
    calc_hours = current_date_time.hours + (calc_minutes / 60);

    calc_hours = calc_hours % 24;
    calc_minutes = calc_minutes % 60;
    calc_seconds = calc_seconds % 60;

    DEBUG_PR_TRACE("Current time: %02u:%02u:%02u, Alarm: %02lu:%02lu:%02lu", current_date_time.hours, current_date_time.minutes, current_date_time.seconds, calc_hours, calc_minutes, calc_seconds);

    alarm_handle.AlarmTime.Hours = calc_hours;
    alarm_handle.AlarmTime.Minutes = calc_minutes;
    alarm_handle.AlarmTime.Seconds = calc_seconds;
    alarm_handle.AlarmTime.SubSeconds = 0x0;
    alarm_handle.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    alarm_handle.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
    alarm_handle.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY; // HH:MM:SS match
    alarm_handle.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
    alarm_handle.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
    alarm_handle.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY; // Nonspecific
    alarm_handle.Alarm = RTC_ALARM_A;
    HAL_RTC_SetAlarm_IT(&rtc_handle, &alarm_handle, RTC_FORMAT_BIN);

    return SYSHAL_RTC_NO_ERROR;
}

__attribute__((weak)) void syshal_timer_callback(void)
{
    DEBUG_PR_WARN("%s() Not implemented", __FUNCTION__);
}

void HAL_RTC_MspInit(RTC_HandleTypeDef * rtcHandle)
{
    if (rtcHandle->Instance == RTC)
    {
        __HAL_RCC_RTC_ENABLE();

        HAL_NVIC_SetPriority(RTC_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(RTC_IRQn);
    }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef * rtcHandle)
{
    if (rtcHandle->Instance == RTC)
    {
        __HAL_RCC_RTC_DISABLE();

        HAL_NVIC_DisableIRQ(RTC_IRQn);
    }
}

void RTC_IRQHandler(void)
{
    HAL_RTC_AlarmIRQHandler(&rtc_handle);
    HAL_RTCEx_WakeUpTimerIRQHandler(&rtc_handle); // NOTE: is this needed?
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef * hrtc)
{
    UNUSED(hrtc);

    syshal_timer_callback();
}