/* syshal_switch.h - HAL for saltwater switch
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

#ifndef _SYSHAL_SWITCH_H_
#define _SYSHAL_SWITCH_H_

#include <stdint.h>
#include <stdbool.h>

#define SYSHAL_SWITCH_NO_ERROR  (0)

int syshal_switch_init(void);
bool syshal_switch_get(void);

#endif /* _SYSHAL_SWITCH_H_ */