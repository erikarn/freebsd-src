#!/bin/sh

. local/opts.inc

env MACHINE=arm ${X_SRCDIR}/sys/tools/fdt/make_dtb.sh \
    ${X_SRCDIR}/sys \
    ${X_SRCDIR}/sys/dts/arm/qcom-ipq4018-rt-ac58u.dts \
    ..
