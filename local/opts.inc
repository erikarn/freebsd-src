
# X_TCHAIN=llvm12
# X_CROSS_TOOLCHAIN_ENTRY="CROSS_TOOLCHAIN=${X_TCHAIN}"

X_CROSS_TOOLCHAIN_ENTRY=""

X_MAKEOPTS="-j8 -s"

X_PREFIX=/data/1/adrian/freebsd/head-embedded-arm/

X_SRCDIR=${X_PREFIX}/freebsd-src
X_OBJDIR=${X_PREFIX}/freebsd-obj
X_ROOTDIR=${X_PREFIX}/freebsd-root
