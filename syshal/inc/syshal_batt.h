/* syshal_batt.h - HAL for battery monitoring
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

#ifndef _SYSHAL_BATT_H_
#define _SYSHAL_BATT_H_

#include <stdint.h>

typedef enum
{
    POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN,
    POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL,
    POWER_SUPPLY_CAPACITY_LEVEL_LOW,
    POWER_SUPPLY_CAPACITY_LEVEL_NORMAL,
    POWER_SUPPLY_CAPACITY_LEVEL_HIGH,
    POWER_SUPPLY_CAPACITY_LEVEL_FULL,
} syshal_batt_state_t;

void syshal_batt_init(uint32_t instance);
uint16_t syshal_batt_temp(void);
uint16_t syshal_batt_voltage(void);
syshal_batt_state_t syshal_batt_state(void);

#endif /* _SYSHAL_BATT_H_ */