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

# DEVICE_VENDOR ASUS
# DEVICE_MODEL RT-AC58U
# SOC qcom-ipq4018
# BLOCKSIZE 128k
# PAGESIZE 2058
# IMAGE_SIZE 20439364
# FILESYSTEMS squashfs
# UIMAGE_MAGIC 0x27051956
# (which requires -M as an option to mkimage?)
# UIMAGE_NAME $(shell echo -e '\03\01\01\01RT-AC58U')

echo "*** making a legacy image"
mkimage \
	-A arm \
	-O linux \
	-T kernel \
	-C lzma \
	-a 0x82000000 \
	-e 0x82000200 \
	-n `echo -e '\03\01\01\01RT-AC58U'` \
	-d ../kernel.elf.lzma \
	../image.trx

echo "*** making a nested image"
mkimage \
	-A arm \
	-O linux \
	-T kernel \
	-C none \
	-a 0x82000000 \
	-e 0x82000200 \
	-n `echo -e '\03\01\01\01RT-AC58U'` \
	-d ../test.itb \
	../image2.trx

echo "*** Copying into /tftpboot"
cp -vf ../test.itb /tftpboot
cp -vf ../image.trx /tftpboot
cp -vf ../image2.trx /tftpboot
cp -vf ../kernel.elf /tftpboot
cp -vf ../qcom-ipq4018-rt-ac58u.dtb /tftpboot/kernel.dtb

