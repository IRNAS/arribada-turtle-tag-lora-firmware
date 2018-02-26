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

#ifndef _SYSHAL_I2C_H_
#define _SYSHAL_I2C_H_

void syshal_i2c_init(void * settings);
void syshal_i2c_term(void);
void syshal_i2c_transfer(uint8_t * data, uint32_t length);
uint32_t syshal_i2c_receive(uint8_t * data); // returns length of data read

begin()
requestFrom()
beginTransmission()
endTransmission()
write()
available()
read()
SetClock()
onReceive()
onRequest()

#endif /* _SYSHAL_I2C_H_ */