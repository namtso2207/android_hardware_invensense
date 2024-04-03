#!/bin/sh
ROOT="/"
install -m 0644 -t "${ROOT}/lib/firmware" inv_dmpfirmware.bin
install -m 0755 -t "${ROOT}/usr/lib" libinvnsensors.so
install -m 0755 -t "${ROOT}/usr/bin" test-sensors-hal
