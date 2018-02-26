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

#ifndef _SYSHAL_TIME_H_
#define _SYSHAL_TIME_H_

uint32_t GetTicksMs(void);

#define TICKS_PER_SECOND ( 1000 )
#define ROUND_NEAREST_MULTIPLE(value, magnitude) ( (value + magnitude / 2) / magnitude ) // Integer round to nearest manitude
#define TIME_IN_SECONDS ( ROUND_NEAREST_MULTIPLE(GetTicksMs(), TICKS_PER_SECOND) ) // Convert millisecond time to seconds

#endif /* _SYSHAL_TIME_H_ */