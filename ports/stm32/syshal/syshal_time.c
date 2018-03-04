/**
  ******************************************************************************
  * @file     syshal_time.c
  * @brief    System hardware abstraction layer for system time.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2018 Arribada</center></h2>
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
  *
  ******************************************************************************
  */

#include "stm32f0xx_hal.h"
#include "syshal_time.h"

/**
 * @brief      Get time since power on in ms
 *
 * @return     Time in milliseconds
 */
uint32_t syshal_time_get_ticks_ms(void)
{
    return HAL_GetTick();
}

/**
 * @brief      Wait for a given time (blocking)
 *
 * @param[in]  ms    Time in milliseconds
 */
void syshal_time_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}