/* syshal_time.h - HAL for time keeping
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

#ifndef _SYSHAL_TIME_H_
#define _SYSHAL_TIME_H_

uint32_t GetTicksMs(void);

#define TICKS_PER_SECOND ( 1000 )
#define ROUND_NEAREST_MULTIPLE(value, magnitude) ( (value + magnitude / 2) / magnitude ) // Integer round to nearest manitude
#define TIME_IN_SECONDS ( ROUND_NEAREST_MULTIPLE(GetTicksMs(), TICKS_PER_SECOND) ) // Convert millisecond time to seconds

#endif /* _SYSHAL_TIME_H_ */