/* Copyright (C) 2018 Arribada
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

#include "bsp.h"
#include "syshal_pmu.h"
#include "debug.h"

// Sets the desired power mode
void syshal_pmu_setLevel(power_level_t level)
{

    switch (level)
    {
        case POWER_STOP:
            DEBUG_PR_TRACE("Entering POWER_STOP mode");
            HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
            break;

        case POWER_SLEEP:
            DEBUG_PR_TRACE("Entering POWER_SLEEP mode");
            HAL_PWR_EnterSLEEPMode(0, PWR_SLEEPENTRY_WFI);
            break;

        case POWER_STANDBY:
            DEBUG_PR_TRACE("Entering POWER_STANDBY mode");
            HAL_PWR_EnterSTANDBYMode();
            break;

        default:
            break;
    }

}