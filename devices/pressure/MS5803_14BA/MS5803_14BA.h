/* MS5803_14BA.h - Global defines for the MS5803_14BA pressure sensor
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

#ifndef _MS5803_14BA_H_
#define _MS5803_14BA_H_

#define MS5803_I2C_ADDRESS    (0x77)

#define CMD_RESET       (0x1E) // ADC reset command
#define CMD_PROM        (0xA0) // PROM location
#define CMD_ADC_READ    (0x00) // ADC read command
#define CMD_ADC_CONV    (0x40) // ADC conversion command
#define CMD_ADC_D1      (0x00) // ADC D1 conversion
#define CMD_ADC_D2      (0x10) // ADC D2 conversion
#define CMD_ADC_256     (0x00) // ADC resolution=256
#define CMD_ADC_512     (0x02) // ADC resolution=512
#define CMD_ADC_1024    (0x04) // ADC resolution=1024
#define CMD_ADC_2048    (0x06) // ADC resolution=2048
#define CMD_ADC_4096    (0x08) // ADC resolution=4096

#endif /* _MS5803_14BA_H_ */