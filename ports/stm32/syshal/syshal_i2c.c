/**
  ******************************************************************************
  * @file     syshal_i2c.c
  * @brief    System hardware abstraction layer for I2C.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2018 Arribada</center></h2>
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
  *
  ******************************************************************************
  */

#include "stm32f0xx_hal.h"
#include "syshal_gpio.h"
#include "syshal_i2c.h"
#include "bsp.h"
#include "debug.h"

// Private variables
static I2C_HandleTypeDef hi2c1;
//static I2C_HandleTypeDef hi2c2;

/**
 * @brief      Initialise the given I2C instance
 *
 * @param[in]  instance  The I2C instance
 */
void syshal_i2c_init(uint32_t instance)
{
    if (I2C_1 == instance)
    {
        // Populate internal handlers
        hi2c1.Instance = I2C_Inits[I2C_1].Instance;
        hi2c1.Init = I2C_Inits[I2C_1].Init;

        HAL_I2C_Init(&hi2c1);
    }

    /*if (I2C_2 == instance)
    {
        // Populate internal handlers
        hi2c2.Instance = I2C_Inits[I2C_2].Instance;
        hi2c2.Init = I2C_Inits[I2C_2].Init;

        HAL_I2C_Init(&hi2c2);
    }*/
}

/**
 * @brief      Transfer the given data to a slave
 *
 * @param[in]  instance      The I2C instance
 * @param[in]  slaveAddress  The slave 8-bit address
 * @param[in]  data          The data buffer to be sent
 * @param[in]  size          The size of the data buffer in bytes
 */
void syshal_i2c_transfer(uint32_t instance, uint8_t slaveAddress, uint8_t * data, uint32_t size)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    if (I2C_1 == instance)
        status = HAL_I2C_Master_Transmit(&hi2c1, slaveAddress, data, size, I2C_TIMEOUT);

    /*if (I2C_2 == instance)
        status = HAL_I2C_Master_Transmit(&hi2c2, slaveAddress, data, size, I2C_TIMEOUT);*/

    if (HAL_OK != status)
    {
        DEBUG_PR_ERROR("%s failed with %d", __FUNCTION__, status);
    }
}

/**
 * @brief      Receive the given data from a slave
 *
 * @param[in]  instance      The I2C instance
 * @param[in]  slaveAddress  The slave 8-bit address
 * @param[out] data          The data buffer to be read into
 * @param[in]  size          The size of the data to be read in bytes
 *
 * @return     The number of bytes read
 */
uint32_t syshal_i2c_receive(uint32_t instance, uint8_t slaveAddress, uint8_t * data, uint32_t size)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    if (I2C_1 == instance)
        status = HAL_I2C_Master_Receive(&hi2c1, slaveAddress, data, size, I2C_TIMEOUT);

    /*if (I2C_2 == instance)
        status = HAL_I2C_Master_Receive(&hi2c2, slaveAddress, data, size, I2C_TIMEOUT);*/

    // return number of bytes read as best we can tell
    if (HAL_OK != status)
    {
        DEBUG_PR_ERROR("%s failed with %d", __FUNCTION__, status);
        return 0;
    }

    return size;
}

/**
 * @brief      Read the given register
 *
 * @param[in]  instance      The I2C instance
 * @param[in]  slaveAddress  The slave 8-bit address
 * @param[in]  regAddress    The register address
 * @param[out] data          The data buffer to be read into
 * @param[in]  size          The size of the data to be read in bytes
 *
 * @return     The number of bytes read
 */
uint32_t syshal_i2c_read_reg(uint32_t instance, uint8_t slaveAddress, uint8_t regAddress, uint8_t * data, uint32_t size)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    if (I2C_1 == instance)
        status = HAL_I2C_Mem_Read(&hi2c1, slaveAddress, regAddress, size, data, 1, I2C_TIMEOUT);

    /*if (I2C_2 == instance)
        status = HAL_I2C_Mem_Read(&hi2c2, slaveAddress, regAddress, size, data, 1, I2C_TIMEOUT);*/

    // return number of bytes read as best we can tell
    if (HAL_OK != status)
    {
        DEBUG_PR_ERROR("%s failed with %d", __FUNCTION__, status);
        return 0;
    }

    return size;
}

/**
 * @brief      Write to the given register
 *
 * @param[in]  instance      The I2C instance
 * @param[in]  slaveAddress  The slave 8-bit address
 * @param[in]  regAddress    The register address
 * @param[in]  data          The data buffer to be sent
 * @param[in]  size          The size of the data buffer in bytes
 */
void syshal_i2c_write_reg(uint32_t instance, uint8_t slaveAddress, uint8_t regAddress, uint8_t * data, uint32_t size)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    if (I2C_1 == instance)
        status = HAL_I2C_Mem_Write(&hi2c1, slaveAddress, regAddress, size, data, size, I2C_TIMEOUT);

    /*if (I2C_2 == instance)
        status = HAL_I2C_Mem_Write(&hi2c2, slaveAddress, regAddress, size, data, size, I2C_TIMEOUT);*/

    if (HAL_OK != status)
    {
        DEBUG_PR_ERROR("%s failed with %d", __FUNCTION__, status);
    }
}

// Implement MSP hooks that are called by stm32f0xx_hal_i2c
void HAL_I2C_MspInit(I2C_HandleTypeDef * hi2c)
{

    if (hi2c->Instance == I2C1)
    {
        // I2C1 GPIO Configuration
        syshal_gpio_init(GPIO_I2C1_SDA);
        syshal_gpio_init(GPIO_I2C1_SCL);

        // Peripheral clock enable
        __HAL_RCC_I2C1_CLK_ENABLE();

    }
    /*if (hi2c->Instance == I2C2)
    {
        // I2C2 GPIO Configuration
        // syshal_gpio_init(GPIO_I2C2_SDA); // TODO needs implementing
        // syshal_gpio_init(GPIO_I2C2_SCL); // TODO needs implementing

        // Peripheral clock enable
        __HAL_RCC_I2C2_CLK_ENABLE();
    }*/

}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef * hi2c)
{

    if (hi2c->Instance == I2C1)
    {
        // Peripheral clock disable
        __HAL_RCC_I2C1_CLK_DISABLE();

        // I2C1 GPIO Configuration
        syshal_gpio_term(GPIO_I2C1_SDA);
        syshal_gpio_term(GPIO_I2C1_SCL);
    }
    /*if (hi2c->Instance == I2C2)
    {
        // Peripheral clock disable
        __HAL_RCC_I2C2_CLK_DISABLE();

        // I2C2 GPIO Configuration
        // syshal_gpio_term(GPIO_I2C2_SDA); // TODO needs implementing
        // syshal_gpio_term(GPIO_I2C2_SCL); // TODO needs implementing
    }*/

}