#!/bin/sh
OE_RPB_ROOT="/media/jmaneyrol/aba85efa-1428-4487-9c0a-fa5aaba8247c/Dev/oe-rpb"
OE_RPB_SYSROOT="${OE_RPB_ROOT}/build-rpb-wayland/tmp-rpb_wayland-glibc/sysroots"
OE_RPB_HOST="x86_64"
OE_RPB_TARGET="hikey"
OE_RPB_CC_PATH="usr/bin/aarch64-linaro-linux"

export CC="aarch64-linaro-linux-gcc"
export CXX="aarch64-linaro-linux-g++"
export LD="aarch64-linaro-linux-ld"

export PATH="${OE_RPB_SYSROOT}/${OE_RPB_HOST}/${OE_RPB_CC_PATH}:${PATH}"
export CFLAGS="--sysroot=${OE_RPB_SYSROOT}/${OE_RPB_TARGET}"
export CXXFLAGS="$CFLAGS"
