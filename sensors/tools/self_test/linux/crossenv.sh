#!/bin/sh
OE_RPB_PATH="${HOME}/oe-rpb/build-rpb-wayland/tmp-rpb_wayland-glibc"
OE_RPB_KERNEL="hikey-linaro-linux/linux-generic-stable/4.14+gitAUTOINC+bebc6082da-r0"
OE_RPB_CC_PATH="usr/bin/aarch64-linaro-linux"

export CC="aarch64-linaro-linux-gcc"
export LD="aarch64-linaro-linux-ld"

export PATH="${OE_RPB_PATH}/work/${OE_RPB_KERNEL}/recipe-sysroot-native/${OE_RPB_CC_PATH}:${PATH}"
export CFLAGS="--sysroot=${OE_RPB_PATH}/work/${OE_RPB_KERNEL}/recipe-sysroot"
