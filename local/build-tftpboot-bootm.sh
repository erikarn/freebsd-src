#!/bin/sh

. local/opts.inc

cd ${X_SRCDIR} || exit 1

#echo "*** Making dtb"
env MACHINE=arm sys/tools/fdt/make_dtb.sh ${X_PREFIX}/freebsd-src/sys ${X_PREFIX}/freebsd-src/sys/dts/arm/qcom-ipq4018-rt-ac58u.dts ..

echo "*** Copying kernel"
cp ${X_OBJDIR}/data/1/adrian/freebsd/head-embedded-arm/freebsd-src/arm.armv7/sys/ASUS_AC1300_MFSROOT/kernel ../kernel-mfsroot.elf
cp ${X_OBJDIR}/data/1/adrian/freebsd/head-embedded-arm/freebsd-src/arm.armv7/sys/ASUS_AC1300_MFSROOT/kernel.debug ../kernel-mfsroot.debug

# TODO: this doesn't work; I have a lot more stuff to do for the DTB!
echo "*** empty initrd"
dd if=/dev/zero of=../ramdisk.bin bs=1024 count=64

echo "*** Building FIT"
mkimage -f local/test-bootm.its ../test-bootm.itb

echo "*** Copying into /tftpboot"
cp -vf ../test-bootm.itb /tftpboot
cp -vf ../kernel-mfsroot.elf /tftpboot
cp -vf ../qcom-ipq4018-rt-ac58u.dtb /tftpboot/kernel-mfsroot.dtb

