#!/bin/sh

TCHAIN=llvm12
CROSS_TOOLCHAIN_ENTRY="CROSS_TOOLCHAIN=${TCHAIN}"
# CROSS_TOOLCHAIN=""

# env MAKEOBJDIRPREFIX=/home/adrian/work/freebsd/head-embedded-arm/freebsd-obj make -j3 KERNCONF=ASUS_AC1300 TARGET_ARCH=armv7 NO_CLEAN=1 SRCCONF=../src.conf MODULES_OVERRIDE="" UBLDR_LOADADDR='0x80208000' SRC_ENV_CONF=../src-env.conf buildworld
env MAKEOBJDIRPREFIX=/home/adrian/work/freebsd/head-embedded-arm/freebsd-obj make -j3 KERNCONF=ASUS_AC1300 TARGET_ARCH=armv7 NO_CLEAN=1 SRCCONF=../src.conf MODULES_OVERRIDE="" UBLDR_LOADADDR="0x80208000" SRC_ENV_CONF=../src-env.conf buildkernel
