#!/bin/sh

. ./local/opts.inc

cd ${X_SRCDIR} || exit 1

env MAKEOBJDIRPREFIX=${X_OBJDIR} make -s -j3 KERNCONF=ASUS_AC1300 \
    TARGET_ARCH=armv7 NO_CLEAN=1 ${X_CROSS_TOOLCHAIN_ENTRY} \
    SRCCONF=src.conf MODULES_OVERRIDE="" UBLDR_LOADADDR='0x80208000' \
    SRC_ENV_CONF=src-env.conf DESTDIR=${X_ROOTDIR} NO_ROOT=YES installworld installkernel distribution
