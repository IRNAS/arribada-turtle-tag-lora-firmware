/* syshal_usb.h - HAL for USB
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

#ifndef _SYSHAL_USB_H_
#define _SYSHAL_USB_H_

#include <stdint.h>

// Constants
#define SYSHAL_USB_NO_ERROR            0
#define SYSHAL_USB_ERROR_BUSY         -1
#define SYSHAL_USB_ERROR_FAIL         -2
#define SYSHAL_USB_ERROR_DISCONNECTED -3

int syshal_usb_init(void);
int syshal_usb_term(void);
int syshal_usb_transfer(uint8_t * data, uint32_t size);
int syshal_usb_receive(uint8_t * data, uint32_t size);
int syshal_usb_available(void);

#endif /* _SYSHAL_USB_H_ */