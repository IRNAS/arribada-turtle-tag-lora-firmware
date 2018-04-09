basedir=$(top_srcdir)/..
core_srcdir=$(basedir)/core
devices_srcdir=$(basedir)/devices
syshal_srcdir=$(basedir)/syshal
gtest_srcdir=$(top_srcdir)/gtest
hw_platform=arribada_v1
cpu_platform=stm32

AM_CPPFLAGS = -I$(syshal_srcdir)/inc \
              -I$(top_srcdir)/gtest/mocks \
              -g -Wall -DUNITTESTING -O0 \
              -DBASEDIR=\"$(abspath $(gtest_srcdir))/\"

HW_DEFS =
AM_CXXFLAGS = -std=c++0x -g -pg -fprofile-arcs -ftest-coverage \
              -I$(GTEST_INCLUDE) -O0 $(HW_DEFS) -DGTEST

AM_CFLAGS = -std=gnu99 -fprofile-arcs -ftest-coverage -O0 $(HW_DEFS) -DGTEST -DCMOCK_MEM_SIZE=0x400000

COMMON_LDADD = -L$(GTEST_LIBS) -lgtest -lgtest_main -lpthread -lgcov -lrt

if BUILD_UNITTEST
EXT_TEST_SRCS =
TESTS =
TESTS_ENVIRONMENT = SRCDIR=$(top_srcdir)
check_PROGRAMS =

check_PROGRAMS += Fs
nodist_Fs_SOURCES = $(gtest_srcdir)/Fs.cpp \
                    $(gtest_srcdir)/mocks/cmock.c \
                    $(gtest_srcdir)/mocks/Mocksyshal_flash.c \
                    $(core_srcdir)/fs/fs.c

Fs_LDADD = $(COMMON_LDADD)
Fs_CPPFLAGS = $(AM_CPPFLAGS) -I$(core_srcdir)/fs

TESTS += Fs.test

check_PROGRAMS += Flash
nodist_Flash_SOURCES = $(gtest_srcdir)/Flash.cpp \
                    $(gtest_srcdir)/mocks/cmock.c \
                    $(gtest_srcdir)/mocks/Mocksyshal_spi.c \
                    $(devices_srcdir)/flash/S25FL128/S25FL128.c

Flash_LDADD = $(COMMON_LDADD)
Flash_CPPFLAGS = $(AM_CPPFLAGS) -I$(devices_srcdir)/flash/S25FL128

TESTS += Flash.test

check_PROGRAMS += Ring_Buffer
nodist_Ring_Buffer_SOURCES = $(gtest_srcdir)/Ring_Buffer.cpp

Ring_Buffer_LDADD = $(COMMON_LDADD)
Ring_Buffer_CPPFLAGS = $(AM_CPPFLAGS) -I$(core_srcdir)/ring_buffer
TESTS += Ring_Buffer.test

check_PROGRAMS += Ble_nRF52
nodist_Ble_nRF52_SOURCES = $(gtest_srcdir)/Ble_nRF52.cpp \
                    $(gtest_srcdir)/mocks/cmock.c \
                    $(gtest_srcdir)/mocks/Mocksyshal_spi.c \
                    $(devices_srcdir)/ble/nRF52x/syshal_ble.c

Ble_nRF52_LDADD = $(COMMON_LDADD)
Ble_nRF52_CPPFLAGS = $(AM_CPPFLAGS) -I$(devices_srcdir)/ble/nRF52x
TESTS += Ble_nRF52.test

LINT_FLAGS  = -"A(C++2010)"
LINT_FLAGS += -e537 # Ignore repeated includes
LINT_FLAGS += -e160 # Allow GNU statement expression extension
LINT_FLAGS += -"efile(451,/usr/include/*)"
LINT_FLAGS += -"efile(451,stdarg.h)"
LINT_FLAGS += -"efile(451,stdarg.h)"
LINT_FLAGS += -"e747" # Significant prototype coercion - temporary?
LINT_FLAGS += +ffn -"width(0)" -hf1 -"format=%f(%l): %t %n: %m" # PC-Lint compatible output
LINT_FLAGS += -i$(builddir)/..
LINT_FLAGS += -i$(top_srcdir)
LINT_FLAGS += -i$(top_srcdir)/lib
LINT_FLAGS += -i$(top_srcdir)/tests/mocks
LINT_FLAGS += co-gcc.lnt
LINT_FLAGS += -w3
LINT_FLAGS += -DGTEST_USE_OWN_TR1_TUPLE=1

LOBS =

flint: $(LOBS)

clean-local:
	rm -rf gtest
	rm -f *.lob mocks/Mock*.{c,h}
	rm -f *.log *.xml *.gcda *.gcno *.out
endif
