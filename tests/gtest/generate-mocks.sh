#!/bin/bash

# 
# Run this script to regenerate mock code found in the /mocks directory
# with the exception of cmock.c, cmock.h and unity.h which are
# invariant and taken from ThrowTheSwitch.org.
#
# NOTE: Do not modify the code that is auto-generated since any modifications
# will get overwritten!
#
# NOTE: Assumes you have ruby already installed
#
mocks="../../syshal/inc/syshal_flash.h \
      "
tar -zxvf CMock-master.tar.gz
ruby CMock-master/lib/cmock.rb -ocmock_options.yml $mocks
rm -rf CMock-master
