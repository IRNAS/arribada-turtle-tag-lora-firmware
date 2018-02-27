/* BQ27621.h - BQ27621-G1 fuel gauge definitions
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

// Datasheet: http://www.ti.com/product/bq27621-g1

#ifndef _BQ27621_H_
#define _BQ27621_H_

#define BQ27621_ADDR (0xAA)

// Register addresses
#define BQ27621_REG_CTRL                             (0x00)
#define BQ27621_REG_TEMP                             (0x02)
#define BQ27621_REG_VOLT                             (0x04)
#define BQ27621_REG_FLAGS                            (0x06)
#define BQ27621_REG_NOMINAL_AVAILABLE_CAPACITY       (0x08)
#define BQ27621_REG_FULL_AVAILABLE_CAPACITY          (0x0A)
#define BQ27621_REG_REMAINING_CAPACITY               (0x0C)
#define BQ27621_REG_FULL_CHARGE_CAPACITY             (0x0E)
#define BQ27621_REG_EFFECTIVE_CURRENT                (0x10)
#define BQ27621_REG_AVERAGE_POWER                    (0x18)
#define BQ27621_REG_STATE_OF_CHARGE                  (0x1C)
#define BQ27621_REG_INTERNAL_TEMP                    (0x1E)
#define BQ27621_REG_REMAINING_CAPACITY_UNFILTERED    (0x28)
#define BQ27621_REG_REMAINING_CAPACITY_FILTERED      (0x2A)
#define BQ27621_REG_FULL_CHARGE_CAPACITY_UNFILTERED  (0x2C)
#define BQ27621_REG_FULL_CHARGE_CAPACITY_FILTERED    (0x2E)
#define BQ27621_REG_STATE_OF_CHARGE_UNFILTERED       (0x30)
#define BQ27621_REG_OPERATION_CONF                   (0x3A)

// Register sizes
#define BQ27621_REG_CTRL_SIZE                             (2)
#define BQ27621_REG_TEMP_SIZE                             (2)
#define BQ27621_REG_VOLT_SIZE                             (2)
#define BQ27621_REG_FLAGS_SIZE                            (2)
#define BQ27621_REG_NOMINAL_AVAILABLE_CAPACITY_SIZE       (2)
#define BQ27621_REG_FULL_AVAILABLE_CAPACITY_SIZE          (2)
#define BQ27621_REG_REMAINING_CAPACITY_SIZE               (2)
#define BQ27621_REG_FULL_CHARGE_CAPACITY_SIZE             (2)
#define BQ27621_REG_EFFECTIVE_CURRENT_SIZE                (2)
#define BQ27621_REG_AVERAGE_POWER_SIZE                    (2)
#define BQ27621_REG_STATE_OF_CHARGE_SIZE                  (2)
#define BQ27621_REG_INTERNAL_TEMP_SIZE                    (2)
#define BQ27621_REG_REMAINING_CAPACITY_UNFILTERED_SIZE    (2)
#define BQ27621_REG_REMAINING_CAPACITY_FILTERED_SIZE      (2)
#define BQ27621_REG_FULL_CHARGE_CAPACITY_UNFILTERED_SIZE  (2)
#define BQ27621_REG_FULL_CHARGE_CAPACITY_FILTERED_SIZE    (2)
#define BQ27621_REG_STATE_OF_CHARGE_UNFILTERED_SIZE       (2)
#define BQ27621_REG_OPERATION_CONF_SIZE                   (2)

#endif /* _BQ27621_H_ */