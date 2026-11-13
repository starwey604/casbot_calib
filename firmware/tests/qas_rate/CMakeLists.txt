cmake_minimum_required(VERSION 3.20.0)

# Reuse the exact board overlay and runtime configuration of the bridge.
set(CONF_FILE "${CMAKE_CURRENT_LIST_DIR}/../../trical_bridge/prj.conf")
set(DTC_OVERLAY_FILE "${CMAKE_CURRENT_LIST_DIR}/../../trical_bridge/app.overlay")
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(trical_qas_rate LANGUAGES C)
add_subdirectory(../../trical_bridge/lib bridge_shared)
target_sources(app PRIVATE src/main.c)
