/* sm.h - Main state machine
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

#ifndef _SM_H_
#define _SM_H_

#define SM_MESSAGE_INACTIVITY_TIMEOUT_MS (2000)

typedef enum
{
    SM_STATE_BOOT,
    SM_STATE_STANDBY_BATTERY_CHARGING,
    SM_STATE_STANDBY_BATTERY_LEVEL_LOW,
    SM_STATE_STANDBY_LOG_FILE_FULL,
    SM_STATE_STANDBY_PROVISIONING_NEEDED,
    SM_STATE_STANDBY_TRIGGER_PENDING,
    SM_STATE_PROVISIONING,
    SM_STATE_OPERATIONAL,
} sm_state_t;

sm_state_t sm_get_state(void);
void sm_set_state(sm_state_t s);
void sm_iterate(void);

#endif /* _SM_H_ */