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
#include "unity.h"
#include <assert.h>
#include <stdint.h>
#include "sm.h"
#include <stdlib.h>
}

#include "googletest.h"

#include <cstdlib>
#include <cstring>
#include <iostream>

class SmTest : public ::testing::Test
{

    virtual void SetUp()
    {

    }

    virtual void TearDown()
    {

    }

public:

    // Functions
    void Boot()
    {
        // Ignore all calls to these functions


        //sm_iterate();
    }

};

TEST_F(SmTest, StateSet)
{
    sm_set_state(SM_STATE_BOOT);
    EXPECT_EQ(SM_STATE_BOOT, sm_get_state());
}

//TEST_F(SmTest, BootOk)
//{
//    sm_set_state(SM_STATE_BOOT);
//    Boot();
//    EXPECT_EQ(SM_STATE_STANDBY_PROVISIONING_NEEDED, sm_get_state());
//}