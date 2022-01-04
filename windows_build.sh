#!/bin/sh

export LIBUSB1_SRC="/c/Users/yuki/Documents/Pico/tools/libusb"
export HIDAPI_SRC="/c/Users/yuki/Documents/Pico/tools/hidapi"
export OPENOCD_CONFIG="--disable-werror --enable-static --disable-shared"
export C_INCLUDE_PATH="/c/Users/yuki/Documents/Pico/tools/libusb/libusb:/c/Users/yuki/Documents/Pico/tools/hidapi/hidapi"

/c/Users/yuki/Documents/Pico/tools/openocd/contrib/cross-build.sh x86_64-w64-mingw32
