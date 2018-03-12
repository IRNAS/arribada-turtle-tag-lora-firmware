// Fs.cpp - Filesystem unit tests
//
// Copyright (C) 2018 Arribada
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
extern "C" {
#include <assert.h>
#include <stdint.h>
#include "unity.h"
#include "Mocksyshal_spi.h"
#include "syshal_flash.h"
#include "S25FL128.h"
#include <stdlib.h>
}

#include "googletest.h"

#include <cstdlib>
#include <cstring>
#include <iostream>

#include <list>
#include <vector>

using std::list;
using std::vector;

static uint8_t tx_buf[64][1024];
static uint8_t rx_buf[64][1024];
static uint16_t tx_size[64];
static uint16_t rx_size[64];
static unsigned int rx_buf_wr = 0;
static unsigned int rx_buf_rd = 0;
static unsigned int tx_buf_wr = 0;
static unsigned int tx_buf_rd = 0;

class FlashTest : public ::testing::Test {

    virtual void SetUp() {
        Mocksyshal_spi_Init();
        syshal_spi_transfer_StubWithCallback(syshal_spi_transfer_Callback);
    }

    virtual void TearDown() {
        Mocksyshal_spi_Verify();
        Mocksyshal_spi_Destroy();
    }

public:
    static int syshal_spi_transfer_Callback(uint32_t instance, uint8_t *wr_data, uint8_t *rd_data, uint16_t size,
        int cmock_num_calls)
    {

        if (tx_buf_wr == tx_buf_rd)
        {
            printf("[%u] Unexpected transfer: %u\n", tx_buf_rd);
            assert(0);
        }

        if (tx_size[tx_buf_rd & 63] != size)
        {
            printf("[%u] Unexpected size: actual=%u vs expected=%u\n", tx_buf_rd, size, tx_size[tx_buf_rd & 63]);
            assert(0);
        }

        if (memcmp(tx_buf[tx_buf_rd & 63], wr_data, tx_size[tx_buf_rd & 63]))
        {
            printf("[%u] TX buffer mismatch\n", tx_buf_rd);
            printf("Expected:\n");
            for (unsigned int i = 0; i < size; i++)
                printf("%02x\n", tx_buf[tx_buf_rd & 63][i]);
            printf("Actual:\n");
            for (unsigned int i = 0; i < size; i++)
                printf("%02x\n", wr_data[i]);
            assert(0);
        }

        memcpy(rd_data, rx_buf[rx_buf_rd & 63], rx_size[rx_buf_rd & 63]);

        tx_buf_rd++;
        rx_buf_rd++;

        return SYSHAL_SPI_NO_ERROR;
    }

    void WriteEnable() {
        uint8_t wren[] = { WREN };
        ExpectSpiTransfer(wren, NULL, 1);
    }

    void ExpectSectorErase(uint32_t addr) {
        uint8_t sector_erase[] = { SE, addr >> 16, addr >> 8, addr };
        ExpectSpiTransfer(sector_erase, NULL, 4);
    }

    void ExpectStatus(uint8_t status) {
        uint8_t status_out[] = { RDSR, 0 };
        uint8_t status_in[] = { RDSR, status };
        ExpectSpiTransfer(status_out, status_in, 2);
    }

    void ExpectPageProgram(uint8_t *buf, uint32_t addr, uint16_t size) {
        uint8_t pp[1024];
        pp[0] = PP;
        pp[1] = addr >> 16;
        pp[2] = addr >> 8;
        pp[3] = addr;
        memcpy(&pp[4], buf, size);
        ExpectSpiTransfer(pp, NULL, size + 4);
    }

    void ExpectPageRead(uint8_t *buf, uint32_t addr, uint16_t size) {
        uint8_t out[1024], in[1024];
        memset(out, 0, size + 4);
        out[0] = READ;
        out[1] = addr >> 16;
        out[2] = addr >> 8;
        out[3] = addr;
        in[0] = READ;
        in[1] = addr >> 16;
        in[2] = addr >> 8;
        in[3] = addr;
        memcpy(&in[4], buf, size);
        ExpectSpiTransfer(out, in, size + 4);
    }

    void ExpectSpiTransfer(uint8_t *write, uint8_t *read, uint16_t size) {
        memcpy(tx_buf[tx_buf_wr & 63], write, size);
        tx_size[tx_buf_wr++ & 63] = size;
        if (read)
            memcpy(rx_buf[rx_buf_wr & 63], read, size);
        rx_size[rx_buf_wr++ & 63] = size;
    }

};

TEST_F(FlashTest, FlashInitOk)
{
    syshal_spi_init_ExpectAndReturn(0, SYSHAL_SPI_NO_ERROR);
    EXPECT_EQ(SYSHAL_FLASH_NO_ERROR, syshal_flash_init(0, 0));
}

TEST_F(FlashTest, FlashTermOk)
{
    syshal_spi_init_ExpectAndReturn(1, SYSHAL_SPI_NO_ERROR);
    EXPECT_EQ(SYSHAL_FLASH_NO_ERROR, syshal_flash_init(0, 1));
    syshal_spi_term_ExpectAndReturn(1, SYSHAL_SPI_NO_ERROR);
    EXPECT_EQ(SYSHAL_FLASH_NO_ERROR, syshal_flash_term(0));
}

TEST_F(FlashTest, FlashInitBadDriveNumber)
{
    EXPECT_EQ(SYSHAL_FLASH_ERROR_INVALID_DRIVE, syshal_flash_init(1, 0));
}

TEST_F(FlashTest, FlashTermBadDriveNumber)
{
    EXPECT_EQ(SYSHAL_FLASH_ERROR_INVALID_DRIVE, syshal_flash_init(1, 0));
}

TEST_F(FlashTest, FlashEraseSector)
{
    syshal_spi_init_ExpectAndReturn(0, SYSHAL_SPI_NO_ERROR);
    EXPECT_EQ(SYSHAL_FLASH_NO_ERROR, syshal_flash_init(0, 0));
    WriteEnable();
    ExpectSectorErase(63 * S25FL128_SECTOR_SIZE);
    ExpectStatus(0);
    EXPECT_EQ(SYSHAL_FLASH_NO_ERROR, syshal_flash_erase(0, 63 * S25FL128_SECTOR_SIZE, S25FL128_SECTOR_SIZE));
}

TEST_F(FlashTest, FlashProgram)
{
    syshal_spi_init_ExpectAndReturn(0, SYSHAL_SPI_NO_ERROR);
    EXPECT_EQ(SYSHAL_FLASH_NO_ERROR, syshal_flash_init(0, 0));
    uint8_t data[] = "DEADBEEF";
    WriteEnable();
    ExpectPageProgram(data, 63 * S25FL128_SECTOR_SIZE, 8);
    ExpectStatus(0);
    EXPECT_EQ(SYSHAL_FLASH_NO_ERROR, syshal_flash_write(0, data, 63 * S25FL128_SECTOR_SIZE, 8));
}

TEST_F(FlashTest, FlashRead)
{
    syshal_spi_init_ExpectAndReturn(0, SYSHAL_SPI_NO_ERROR);
    EXPECT_EQ(SYSHAL_FLASH_NO_ERROR, syshal_flash_init(0, 0));
    uint8_t test_data[] = "DEADBEEF";
    uint8_t actual_data[512];
    ExpectPageRead(test_data, 63 * S25FL128_SECTOR_SIZE, 8);
    EXPECT_EQ(SYSHAL_FLASH_NO_ERROR, syshal_flash_read(0, actual_data, 63 * S25FL128_SECTOR_SIZE, 8));
    EXPECT_EQ(0, memcmp(test_data, actual_data, 8));
}