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

#ifndef _DEBUG_H_
#define _DEBUG_H_

#include <stdio.h>
#include "syshal_time.h"

typedef enum
{
    DEBUG_NONE,
    DEBUG_SYSTEM,
    DEBUG_ERROR,
    DEBUG_WARN,
    DEBUG_INFO,
    DEBUG_TRACE,
} debug_level_t;

extern const char * g_dbg_lvl[];
extern debug_level_t g_debug_level;

#ifndef DEBUG_DISABLED

#define DEBUG_PR(fmt, ...)     printf(fmt "\n\r", ## __VA_ARGS__)

#ifndef DEBUG_COLOR

#define DEBUG_PR_T(lvl, fmt, ...)   if (g_debug_level >= lvl) { \
    DEBUG_PR("%lu\t%s\t" fmt, \
    TIME_IN_SECONDS, \
    g_dbg_lvl[lvl], ## __VA_ARGS__); \
}

#define DEBUG_PR_SYS(fmt, ...)       DEBUG_PR_T(DEBUG_SYSTEM, fmt, ## __VA_ARGS__)
#define DEBUG_PR_INFO(fmt, ...)      DEBUG_PR_T(DEBUG_INFO, fmt, ## __VA_ARGS__)
#define DEBUG_PR_WARN(fmt, ...)      DEBUG_PR_T(DEBUG_WARN, fmt, ## __VA_ARGS__)
#define DEBUG_PR_ERROR(fmt, ...)     DEBUG_PR_T(DEBUG_ERROR, fmt, ## __VA_ARGS__)
#define DEBUG_PR_TRACE(fmt, ...)     DEBUG_PR_T(DEBUG_TRACE, fmt, ## __VA_ARGS__)

#else

#define DEBUG_PR_T(lvl, color, fmt, ...)   if (g_debug_level >= lvl) { \
    DEBUG_PR("\e[39;49m" "%lu\t" color "%s\t" fmt, \
    TIME_IN_SECONDS, \
    g_dbg_lvl[lvl], ## __VA_ARGS__); \
}

#define DEBUG_PR_SYS(fmt, ...)       DEBUG_PR_T(DEBUG_SYSTEM, "", fmt, ## __VA_ARGS__)
#define DEBUG_PR_INFO(fmt, ...)      DEBUG_PR_T(DEBUG_INFO, "\e[38;5;4m", fmt, ## __VA_ARGS__)
#define DEBUG_PR_WARN(fmt, ...)      DEBUG_PR_T(DEBUG_WARN, "\e[38;5;130m", fmt, ## __VA_ARGS__)
#define DEBUG_PR_ERROR(fmt, ...)     DEBUG_PR_T(DEBUG_ERROR, "\e[38;5;1m", fmt, ## __VA_ARGS__)
#define DEBUG_PR_TRACE(fmt, ...)     DEBUG_PR_T(DEBUG_TRACE, "\e[38;5;8m", fmt, ## __VA_ARGS__)

#endif

#else

#define DEBUG_PR(fmt, ...)
#define DEBUG_PR_T(lvl, fmt, ...)
#define DEBUG_PR_SYS(fmt, ...)
#define DEBUG_PR_INFO(fmt, ...)
#define DEBUG_PR_WARN(fmt, ...)
#define DEBUG_PR_ERROR(fmt, ...)
#define DEBUG_PR_TRACE(fmt, ...)

#endif

#endif /* _DEBUG_H_ */

