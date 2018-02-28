/* syshal_flash.h - HAL for flash device
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

#ifndef _SYSHAL_FLASH_H_
#define _SYSHAL_FLASH_H_

#include <stdint.h>

/* Constants */

/* Macros */

/* Types */

/* Functions */

int syshal_flash_init(uint32_t device);
int syshal_flash_term(uint32_t device);
int syshal_flash_erase(uint32_t device, uint32_t address, uint32_t size);
int syshal_flash_write(uint32_t device, const void *src, uint32_t address, uint32_t size);
int syshal_flash_read(uint32_t device, void * const dest, uint32_t address, uint32_t size);

#endif /* _SYSHAL_FLASH_H_ */
