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
#include <stdint.h>
#include "unity.h"
#include "Mocksyshal_flash.h"
#include "fs.h"
#include "fs_priv.h"
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

#define FLASH_SIZE          (FS_PRIV_SECTOR_SIZE * FS_PRIV_MAX_SECTORS)
#define ASCII(x)            ((x) >= 32 && (x) <= 127) ? (x) : '.'

static char flash_ram[FLASH_SIZE];

class FsTest : public ::testing::Test {

    virtual void SetUp() {
        Mocksyshal_flash_Init();
        syshal_flash_read_StubWithCallback(syshal_flash_read_Callback);
        syshal_flash_write_StubWithCallback(syshal_flash_write_Callback);
        syshal_flash_erase_StubWithCallback(syshal_flash_erase_Callback);
        for (unsigned int i = 0; i < FLASH_SIZE; i++)
            flash_ram[i] = 0xFF;
    }

    virtual void TearDown() {
        Mocksyshal_flash_Verify();
        Mocksyshal_flash_Destroy();
    }

public:
    void CheckSectorAllocCounter(uint8_t sector, uint32_t alloc_counter) {
        union {
            uint32_t *alloc_counter;
            char *buffer;
        } a;
        a.buffer = &flash_ram[(sector * FS_PRIV_SECTOR_SIZE) + FS_PRIV_ALLOC_COUNTER_OFFSET];
        EXPECT_EQ(alloc_counter, *a.alloc_counter);
    }

    void CheckFileId(uint8_t sector, uint8_t file_id)
    {
        EXPECT_EQ(file_id, flash_ram[(sector * FS_PRIV_SECTOR_SIZE)]);
    }

    void DumpFlash(uint32_t start, uint32_t sz) {
        for (unsigned int i = 0; i < sz/8; i++) {
            printf("%08x:", start + (8*i));
            for (unsigned int j = 0; j < 8; j++)
                printf(" %02x", (unsigned char)flash_ram[start + (8*i) + j]);
            printf("  ");
            for (unsigned int j = 0; j < 8; j++)
                printf("%c", ASCII((unsigned char)flash_ram[start + (8*i) + j]));
            printf("\n");
        }
    }

    static int syshal_flash_read_Callback(uint32_t device, void *dest, uint32_t address, uint32_t size, int cmock_num_calls)
    {
        //printf("syshal_flash_read(%08x,%u)\n", address, size);
        for (unsigned int i = 0; i < size; i++)
            ((char *)dest)[i] = flash_ram[address + i];

        return 0;
    }

    static int syshal_flash_write_Callback(uint32_t device, const void *src, uint32_t address, uint32_t size, int cmock_num_calls)
    {
        for (unsigned int i = 0; i < size; i++)
        {
            /* Ensure no new bits are being set */
            if ((((char *)src)[i] & flash_ram[address + i]) ^ ((char *)src)[i])
            {
                printf("syshal_flash_write: Can't set bits from 0 to 1 (%08x: %02x => %02x)\n", i,
                        flash_ram[address + i], ((char *)src)[i]);
                assert(0);
            }
            flash_ram[address + i] = ((char *)src)[i];
        }

        return 0;
    }

    static int syshal_flash_erase_Callback(uint32_t device, uint32_t address, uint32_t size, int cmock_num_calls)
    {
        /* Make sure address is sector aligned */
        if (address % FS_PRIV_SECTOR_SIZE || size % FS_PRIV_SECTOR_SIZE)
        {
            printf("syshal_flash_erase: Non-aligned address %08x", address);
            assert(0);
        }

        for (unsigned int i = 0; i < size; i++)
            flash_ram[address + i] = 0xFF;

        return 0;
    }
};

TEST_F(FsTest, FormatPreservesAllocationCounter)
{
    fs_t fs;

    syshal_flash_init_ExpectAndReturn(0, 0);
    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    for (unsigned int i = 0; i < FS_PRIV_MAX_SECTORS; i++)
        CheckSectorAllocCounter(i, 0);
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    for (unsigned int i = 0; i < FS_PRIV_MAX_SECTORS; i++)
        CheckSectorAllocCounter(i, 1);
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    for (unsigned int i = 0; i < FS_PRIV_MAX_SECTORS; i++)
        CheckSectorAllocCounter(i, 2);
}

TEST_F(FsTest, SimpleFileIO)
{
    fs_t fs;
    fs_handle_t handle;
    uint32_t wr, rd;
    char test_string[][256] = {
            "Hello World",
    };
    char buf[256];

    syshal_flash_init_ExpectAndReturn(0, 0);
    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_read(handle, buf, sizeof(buf), &rd));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), rd);
    EXPECT_EQ(0, strncmp(test_string[0], buf, strlen(test_string[0])));
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
}

TEST_F(FsTest, FileUserFlagsArePreserved)
{
    fs_t fs;
    fs_handle_t handle;
    uint32_t wr;
    char test_string[][256] = {
            "Hello World",
    };
    uint8_t wr_user_flags = 0x7, rd_user_flags;

    syshal_flash_init_ExpectAndReturn(0, 0);
    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, &wr_user_flags));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_READONLY, &rd_user_flags));
    EXPECT_EQ(wr_user_flags, rd_user_flags);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
}

TEST_F(FsTest, StatExistingFilePreservesAttributes)
{
    fs_t fs;
    fs_handle_t handle;
    fs_stat_t stat;
    uint32_t wr;
    char test_string[][256] = {
            "Hello World",
    };
    uint8_t wr_user_flags = 0x7;

    syshal_flash_init_ExpectAndReturn(0, 0);
    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, &wr_user_flags));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_stat(fs, 0, &stat));
    EXPECT_EQ(wr_user_flags, stat.user_flags);
    EXPECT_FALSE(stat.is_circular);
    EXPECT_FALSE(stat.is_protected);
    EXPECT_EQ((uint32_t)strlen(test_string[0]), stat.size);
}

TEST_F(FsTest, DeletedFileNoLongerExists)
{
    fs_t fs;
    fs_handle_t handle;
    uint32_t wr;
    char test_string[][256] = {
            "Hello World",
    };
    uint8_t wr_user_flags = 0x7;

    syshal_flash_init_ExpectAndReturn(0, 0);
    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, &wr_user_flags));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_delete(fs, 0));
    EXPECT_EQ(FS_ERROR_FILE_NOT_FOUND, fs_open(fs, &handle, 0, FS_MODE_READONLY, NULL));
}

TEST_F(FsTest, FileWriteAppend)
{
    fs_t fs;
    fs_handle_t handle;
    uint32_t wr, rd;
    char test_string[][256] = {
            "Hello World",
            "Hello WorldHello World",
    };
    char buf[256];

    syshal_flash_init_ExpectAndReturn(0, 0);
    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_WRITEONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_read(handle, buf, sizeof(buf), &rd));
    EXPECT_EQ((uint32_t)strlen(test_string[1]), rd);
    EXPECT_EQ(0, strncmp(test_string[1], buf, strlen(test_string[1])));
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
}

TEST_F(FsTest, OpenNonExistentFileExpectFileNotFound)
{
    fs_t fs;
    fs_handle_t handle;

    syshal_flash_init_ExpectAndReturn(0, 0);
    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    for (unsigned int i = 0; i < 256; i++)
        EXPECT_EQ(FS_ERROR_FILE_NOT_FOUND, fs_open(fs, &handle, (uint8_t)i, FS_MODE_READONLY, NULL));
}

TEST_F(FsTest, DeleteNonExistentFileExpectFileNotFound)
{
    fs_t fs;

    syshal_flash_init_ExpectAndReturn(0, 0);
    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    for (unsigned int i = 0; i < 256; i++)
        EXPECT_EQ(FS_ERROR_FILE_NOT_FOUND, fs_delete(fs, (uint8_t)i));
}

TEST_F(FsTest, StatNonExistentFileExpectFileNotFound)
{
    fs_t fs;

    syshal_flash_init_ExpectAndReturn(0, 0);
    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    for (unsigned int i = 0; i < 255; i++)
        EXPECT_EQ(FS_ERROR_FILE_NOT_FOUND, fs_stat(fs, (uint8_t)i, NULL));
}

TEST_F(FsTest, ProtectNonExistentFileExpectFileNotFound)
{
    fs_t fs;

    syshal_flash_init_ExpectAndReturn(0, 0);
    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    for (unsigned int i = 0; i < 256; i++)
        EXPECT_EQ(FS_ERROR_FILE_NOT_FOUND, fs_protect(fs, (uint8_t)i));
}

TEST_F(FsTest, UnprotectNonExistentFileExpectFileNotFound)
{
    fs_t fs;

    syshal_flash_init_ExpectAndReturn(0, 0);
    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    for (unsigned int i = 0; i < 256; i++)
        EXPECT_EQ(FS_ERROR_FILE_NOT_FOUND, fs_unprotect(fs, (uint8_t)i));
}

TEST_F(FsTest, StatEmptyFileSystemExpectMaxCapacityFree)
{
    fs_t fs;
    fs_stat_t stat;

    syshal_flash_init_ExpectAndReturn(0, 0);
    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_stat(fs, FS_FILE_ID_NONE, &stat));
    EXPECT_EQ((uint32_t)FS_PRIV_USABLE_SIZE * FS_PRIV_MAX_SECTORS, stat.size);
}

TEST_F(FsTest, ProtectedFileCannotBeWritten)
{
    fs_t fs;
    fs_handle_t handle;
    uint32_t wr;
    char test_string[][256] = {
            "Hello World",
    };

    syshal_flash_init_ExpectAndReturn(0, 0);
    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_protect(fs, 0));
    EXPECT_EQ(FS_ERROR_FILE_PROTECTED, fs_open(fs, &handle, 0, FS_MODE_WRITEONLY, NULL));
}

TEST_F(FsTest, ProtectedFileCanBeRead)
{
    fs_t fs;
    fs_handle_t handle;
    uint32_t wr, rd;
    char test_string[][256] = {
            "Hello World",
    };
    char buf[256];

    syshal_flash_init_ExpectAndReturn(0, 0);
    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_protect(handle, 0));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_read(handle, buf, sizeof(buf), &rd));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), rd);
    EXPECT_EQ(0, strncmp(test_string[0], buf, strlen(test_string[0])));
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
}

TEST_F(FsTest, ToggledFileProtectionAllowsWrite)
{
    fs_t fs;
    fs_handle_t handle;
    uint32_t wr, rd;
    char test_string[][256] = {
            "Hello World",
            "Hello WorldHello World"
    };
    char buf[256];

    syshal_flash_init_ExpectAndReturn(0, 0);
    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_protect(handle, 0));
    EXPECT_EQ(FS_NO_ERROR, fs_unprotect(handle, 0));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_WRITEONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_read(handle, buf, sizeof(buf), &rd));
    EXPECT_EQ((uint32_t)strlen(test_string[1]), rd);
    EXPECT_EQ(0, strncmp(test_string[1], buf, strlen(test_string[1])));
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
}

TEST_F(FsTest, MultiFileWriteCanReadBackSameData)
{
    fs_t fs;
    fs_handle_t handle;
    uint32_t wr, rd;
    uint8_t user_flags;
    char test_string[][256] = {
            "Hello World",
            "Testing 1, 2, 3"
    };
    char buf[256];

    syshal_flash_init_ExpectAndReturn(0, 0);
    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_ERROR_FILE_NOT_FOUND, fs_open(fs, &handle, 0, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], sizeof(test_string[0]), &wr));
    EXPECT_EQ((uint32_t)sizeof(test_string[0]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 1, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[1], sizeof(test_string[1]), &wr));
    EXPECT_EQ((uint32_t)sizeof(test_string[1]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_read(handle, buf, sizeof(buf), &rd));
    EXPECT_EQ((uint32_t)sizeof(test_string[0]), rd);
    EXPECT_EQ(0, strncmp(test_string[0], buf, sizeof(test_string[0])));
    EXPECT_EQ(FS_ERROR_END_OF_FILE, fs_read(handle, buf, sizeof(buf), &rd));
    EXPECT_EQ((uint32_t)0, rd);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 1, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_read(handle, buf, sizeof(buf), &rd));
    EXPECT_EQ((uint32_t)sizeof(test_string[1]), rd);
    EXPECT_EQ(0, strncmp(test_string[1], buf, sizeof(test_string[1])));
    EXPECT_EQ(FS_ERROR_END_OF_FILE, fs_read(handle, buf, sizeof(buf), &rd));
    EXPECT_EQ((uint32_t)0, rd);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_delete(fs, 1));
    EXPECT_EQ(FS_ERROR_FILE_NOT_FOUND, fs_delete(fs, 1));
    EXPECT_EQ(FS_NO_ERROR, fs_delete(fs, 0));
    EXPECT_EQ(FS_ERROR_FILE_NOT_FOUND, fs_delete(fs, 0));
    user_flags = 0x7;
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 1, FS_MODE_CREATE, &user_flags));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[1], sizeof(test_string[1]), &wr));
    EXPECT_EQ((uint32_t)sizeof(test_string[1]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
}
