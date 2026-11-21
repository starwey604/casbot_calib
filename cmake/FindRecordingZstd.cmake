if(NOT TARGET casbot_calib::zstd)
  find_path(CALIB_ZSTD_INCLUDE_DIR zstd.h)
  find_library(CALIB_ZSTD_LIBRARY NAMES zstd zstd_static libzstd)
  if(NOT CALIB_ZSTD_INCLUDE_DIR OR NOT CALIB_ZSTD_LIBRARY)
    message(FATAL_ERROR "Recording requires libzstd headers and library. On Ubuntu install libzstd-dev.")
  endif()
  add_library(casbot_calib::zstd UNKNOWN IMPORTED)
  set_target_properties(casbot_calib::zstd PROPERTIES
    IMPORTED_LOCATION "${CALIB_ZSTD_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${CALIB_ZSTD_INCLUDE_DIR}")
endif()
