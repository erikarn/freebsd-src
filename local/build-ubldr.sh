#!/bin/sh

CURDIR="`pwd`"

# doesn't work yet?

# env MAKEOBJDIRPREFIX=/home/adrian/work/freebsd/head-embedded-arm/freebsd-obj make -j4 buildworld buildkernel KERNCONF=ASUS_AC1300 CROSS_TOOLCHAIN=llvm10 TARGET_ARCH=armv7 NO_CLEAN=1 SRCCONF=../src.conf UBLDR_LOADADDR=0x80208000
env MAKEOBJDIRPREFIX=/home/adrian/work/freebsd/head-embedded-arm/freebsd-obj make -j4 KERNCONF=ASUS_AC1300 CROSS_TOOLCHAIN=llvm10 TARGET_ARCH=armv7 NO_CLEAN=1 SRCCONF=../src.conf -C "${CURDIR}/stand/arm/uboot" UBLDR_LOADADDR=0x80208000 all
