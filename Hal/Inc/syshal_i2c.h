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

#ifndef _SYSHAL_I2C_H_
#define _SYSHAL_I2C_H_

void syshal_i2c_init(void * settings);
void syshal_i2c_term(void);
void syshal_i2c_transfer(uint8_t * data, uint32_t length);
uint32_t syshal_i2c_receive(uint8_t * data); // returns length of data read

#endif /* _SYSHAL_I2C_H_ */