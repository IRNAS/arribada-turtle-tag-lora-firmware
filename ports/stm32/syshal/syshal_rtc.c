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

#include "syshal_rtc.h"

RTC_HandleTypeDef rtc_handle;

// HAL to SYSHAL error code mapping table
static int hal_error_map[] =
{
    SYSHAL_BLE_NO_ERROR,
    SYSHAL_BLE_ERROR_DEVICE,
    SYSHAL_BLE_ERROR_BUSY,
    SYSHAL_BLE_ERROR_TIMEOUT,
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

    return hal_error_map[status];
}

int syshal_rtc_set_date_and_time(syshal_rtc_data_and_time_t date_time)
{
    HAL_StatusTypeDef status;

    RTC_DateTypeDef date;
    date.WeekDay = RTC_WEEKDAY_MONDAY; // Unused as we don't track the name of the day
    date.Year = data_time.year;
    date.Month = data_time.month;
    date.Date = data_time.day;

    status = HAL_RTC_SetDate(&rtc_handle, &date, RTC_FORMAT_BIN);
    if (HAL_OK != status)
        return hal_error_map[status];

    RTC_TimeTypeDef time;
    time.Hours = data_time.hours;
    time.Minutes = data_time.minutes;
    time.Seconds = data_time.seconds;
    time.SubSeconds = 0;
    time.SecondFraction = 0;
    time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    time.StoreOperation = RTC_STOREOPERATION_SET;

    status = HAL_RTC_SetTime(&rtc_handle, &time, RTC_FORMAT_BIN);

    return hal_error_map[status];
}

int syshal_rtc_get_date_and_time(syshal_rtc_data_and_time_t * data_time)
{
    HAL_StatusTypeDef status;

    RTC_DateTypeDef date;
    status = HAL_RTC_GetDate(&rtc_handle, &date, RTC_FORMAT_BIN);
    if (HAL_OK != status)
        return hal_error_map[status];

    data_time->year = date.Year;
    data_time->month = date.Month;
    data_time->day = date.Date;

    RTC_TimeTypeDef time;
    status = HAL_RTC_GetTime(&rtc_handle, &date, RTC_FORMAT_BIN);
    data_time.hours = time.Hours;
    data_time.minutes = time.Minutes;
    data_time.seconds = time.Seconds;

    return hal_error_map[status];
}

void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{
    if(rtcHandle->Instance==RTC)
        __HAL_RCC_RTC_ENABLE();
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{
    if(rtcHandle->Instance==RTC)
        __HAL_RCC_RTC_DISABLE();
} 