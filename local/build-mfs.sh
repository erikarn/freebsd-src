#!/bin/sh

. ./local/opts.inc

MFS_FILE="mfs-arm.img"
MFS_UZIP_FILE="mfs-arm.uzip"

dd if=/dev/zero of=${MFS_FILE} bs=1m count=8
MDEV="`mdconfig -f ${MFS_FILE}`"

newfs /dev/${MDEV}

mount /dev/${MDEV} /mnt
mkdir -p /mnt/mnt
mkdir -p /mnt/bin
mkdir -p /mnt/sbin
mkdir -p /mnt/lib
mkdir -p /mnt/lib/casper
mkdir -p /mnt/dev
mkdir -p /mnt/etc
mkdir -p /mnt/var
mkdir -p /mnt/proc
mkdir -p /mnt/var/log
mkdir -p /mnt/var/run
mkdir -p /mnt/tmp
chown 777 /mnt/tmp
mkdir -p /mnt/libexec

install -m 0755 -D /mnt ${X_ROOTDIR}/bin/sh /mnt/bin/
install -m 0755 -D /mnt ${X_ROOTDIR}/bin/dd /mnt/bin/
install -m 0755 -D /mnt ${X_ROOTDIR}/bin/ls /mnt/bin/

install -m 0755 -D /mnt ${X_ROOTDIR}/usr/bin/uname /mnt/bin/
install -m 0755 -D /mnt ${X_ROOTDIR}/usr/bin/hexdump /mnt/bin/
install -m 0755 -D /mnt ${X_ROOTDIR}/usr/bin/vmstat /mnt/bin/
install -m 0755 -D /mnt ${X_ROOTDIR}/usr/bin/netstat /mnt/bin/

install -m 0755 -D /mnt ${X_ROOTDIR}/sbin/init /mnt/sbin/
install -m 0755 -D /mnt ${X_ROOTDIR}/sbin/reboot /mnt/sbin/
install -m 0755 -D /mnt ${X_ROOTDIR}/sbin/sysctl /mnt/sbin/
install -m 0755 -D /mnt ${X_ROOTDIR}/sbin/md5 /mnt/sbin/
install -m 0755 -D /mnt ${X_ROOTDIR}/sbin/etherswitchcfg /mnt/sbin/
install -m 0755 -D /mnt ${X_ROOTDIR}/sbin/ifconfig /mnt/sbin/
install -m 0755 -D /mnt ${X_ROOTDIR}/sbin/ping /mnt/sbin/
install -m 0755 -D /mnt ${X_ROOTDIR}/sbin/mount /mnt/sbin/
install -m 0755 -D /mnt ${X_ROOTDIR}/sbin/umount /mnt/sbin/
install -m 0755 -D /mnt ${X_ROOTDIR}/sbin/dmesg /mnt/sbin/
install -m 0755 -D /mnt ${X_ROOTDIR}/sbin/mount_msdosfs /mnt/sbin/

install -m 0755 -D /mnt ${X_ROOTDIR}/usr/sbin/gpioctl /mnt/sbin/

install -m 0755 -D /mnt ${X_ROOTDIR}/libexec/ld-elf.so.1 /mnt/libexec/

install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libedit.so.8 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libncursesw.so.9 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libc.so.7 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libsys.so.7 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libgcc_s.so.1 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libutil.so.9 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libmd.so.7 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libm.so.5 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/lib80211.so.1 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libjail.so.1 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libnv.so.1 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libsbuf.so.6 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libbsdxml.so.4 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libcasper.so.1 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libipsec.so.4 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libkvm.so.7 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libxo.so.0 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libelf.so.2 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libdevstat.so.7 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libtinfow.so.9 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libkiconv.so.4 /mnt/lib/

install -m 0755 -D /mnt ${X_ROOTDIR}/usr/lib/libmemstat.so.3 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/usr/lib/libnetgraph.so.4 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/usr/lib/libgpio.so.0 /mnt/lib/

install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libcap_dns.so.2 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libcap_fileargs.so.1 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libcap_grp.so.1 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libcap_net.so.1 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libcap_netdb.so.1 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libcap_pwd.so.1 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libcap_sysctl.so.2 /mnt/lib/
install -m 0755 -D /mnt ${X_ROOTDIR}/lib/libcap_syslog.so.1 /mnt/lib/

umount /mnt

mdconfig -d -u 0

mkuzip -o /tftpboot/${MFS_UZIP_FILE} ${MFS_FILE}
