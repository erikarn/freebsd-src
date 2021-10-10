#!/bin/sh

TCHAIN=llvm12

# env MAKEOBJDIRPREFIX=/home/adrian/work/freebsd/head-embedded-arm/freebsd-obj make -j4 buildworld buildkernel KERNCONF=ASUS_AC1300 CROSS_TOOLCHAIN=${TCHAIN} TARGET_ARCH=armv7 NO_CLEAN=1 SRCCONF=../src.conf MODULES_OVERRIDE="" UBLDR_LOADADDR='0x80208000"
env MAKEOBJDIRPREFIX=/home/adrian/work/freebsd/head-embedded-arm/freebsd-obj make -j4 buildkernel KERNCONF=ASUS_AC1300 CROSS_TOOLCHAIN=${TCHAIN} TARGET_ARCH=armv7 NO_CLEAN=1 SRCCONF=../src.conf MODULES_OVERRIDE="" UBLDR_LOADADDR="0x80208000"
