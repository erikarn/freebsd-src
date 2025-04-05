#!/bin/sh

. local/opts.inc

cd ${X_SRCDIR} || exit 1

#echo "*** Making dtb"
#sys/tools/fdt/make_dtb.sh /home/adrian/work/freebsd/head-embedded-arm/freebsd-src/sys /usr/home/adrian/work/freebsd/head-embedded-arm/freebsd-src/sys/dts/arm/qcom-ipq4018-rt-ac58u.dts ..

echo "*** Copying kernel"
cp ${X_OBJDIR}/data/1/adrian/freebsd/head-embedded-arm/freebsd-src/arm.armv7/sys/ASUS_AC1300/kernel ../kernel.elf
cp ${X_OBJDIR}/data/1/adrian/freebsd/head-embedded-arm/freebsd-src/arm.armv7/sys/ASUS_AC1300/kernel.debug ../kernel.debug

echo "*** Building FIT"
# mkimage -f local/test.its ../test.itb

echo "*** Copying into /tftpboot"
# cp -vf ../test.itb /tftpboot
cp -vf ../kernel.elf /tftpboot
#cp -vf ../qcom-ipq4018-rt-ac58u.dtb /tftpboot/kernel.dtb
#
