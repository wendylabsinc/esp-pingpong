#!/bin/sh
# Local helper: configure and build the leaf firmware with the ESP-NOW carrier.
set -e
D="sdkconfig.defaults;boards/seeed_xiao_esp32s3_native.defaults;mesh.defaults"
[ -f build-leaf/sdkconfig ] || idf.py -B build-leaf -D SDKCONFIG=build-leaf/sdkconfig -D "SDKCONFIG_DEFAULTS=$D" set-target esp32s3
idf.py -B build-leaf -D SDKCONFIG=build-leaf/sdkconfig -D "SDKCONFIG_DEFAULTS=$D" build
