/* Copyright 2018 Arribada
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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