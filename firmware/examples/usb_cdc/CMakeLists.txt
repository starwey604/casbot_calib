cmake_minimum_required(VERSION 3.20.0)

set(DTC_OVERLAY_FILE "${CMAKE_CURRENT_LIST_DIR}/../common.overlay")
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(casbot_usb_cdc LANGUAGES C)
target_sources(app PRIVATE src/main.c)
