#!/bin/sh

# To use:

# raw kernel
# tftpboot 0x82000000 kernel.elf
# go 0x82000200

# testing the compressed kernel image as a uboot FIT application
# tftpboot 0x84000000 test.itb
# bootm 0x84000000

. local/opts.inc

cd ${X_SRCDIR} || exit 1

#echo "*** Making dtb"
env MACHINE=arm sys/tools/fdt/make_dtb.sh ${X_PREFIX}/freebsd-src/sys ${X_PREFIX}/freebsd-src/sys/dts/arm/qcom-ipq4018-rt-ac58u.dts ..

echo "*** Copying kernel"
cp ${X_OBJDIR}/data/1/adrian/freebsd/head-embedded-arm/freebsd-src/arm.armv7/sys/ASUS_AC1300/kernel ../kernel.elf
cp ${X_OBJDIR}/data/1/adrian/freebsd/head-embedded-arm/freebsd-src/arm.armv7/sys/ASUS_AC1300/kernel.debug ../kernel.debug

# TODO: this doesn't work; I have a lot more stuff to do for the DTB!
echo "*** empty initrd"
dd if=/dev/zero of=../ramdisk.bin bs=1024 count=64

echo "*** compress kernel"
cat ../kernel.elf | lzma > ../kernel.elf.lzma

echo "*** Building FIT"
mkimage -f local/test.its ../test.itb

echo "*** Copying into /tftpboot"
cp -vf ../test.itb /tftpboot
cp -vf ../kernel.elf /tftpboot
cp -vf ../qcom-ipq4018-rt-ac58u.dtb /tftpboot/kernel.dtb

