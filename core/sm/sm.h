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

// Global Defines
typedef enum sm_state_e
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

// Prototypes //
sm_state_t sm_get_state(void);          // Returns the current state
void sm_set_state(sm_state_t state);    // Sets the state
void sm_iterate(void);

#define PARSE_RX_NO_ERROR                    (0)
#define PARSE_RX_ERROR_INSUFFICIENT_BYTES   (-1)
#define PARSE_RX_ERROR_MISSING_SYNC         (-2)
#define PARSE_RX_ERROR_MSG_TOO_BIG          (-3)
#define PARSE_RX_ERROR_MSG_PENDING          (-4)

#endif /* _SM_H_ */