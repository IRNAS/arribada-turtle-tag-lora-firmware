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

#ifndef _SYSHAL_PMU_H_
#define _SYSHAL_PMU_H_

typedef enum
{
    POWER_STOP,    // Lowest power consumption while all the SRAM and registers are kept
    POWER_SLEEP,   // Only the CPU clock is stopped
    POWER_STANDBY, // The lowest power consumption
} power_level_t;

void syshal_pmu_setLevel(power_level_t level);

#endif /* _SYSHAL_PMU_H_ */