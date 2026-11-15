# Included after project() only for the Orin cross preset.
option(CALIB_ORIN_THINLTO "Use ThinLTO for Orin binaries" ON)
if(CALIB_ORIN_THINLTO)
  add_compile_options(-flto=thin)
  add_link_options(-flto=thin "-Wl,--thinlto-cache-dir=${CMAKE_BINARY_DIR}/thinlto-cache")
endif()
add_compile_options(-ffunction-sections -fdata-sections)
add_link_options(-Wl,--gc-sections -Wl,--icf=safe)
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib;/opt/ros/humble/lib/aarch64-linux-gnu;/opt/ros/humble/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)

function(calib_install_orin_openmp component)
  install(FILES /opt/orin-openmp/lib/libomp.so.5
    DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT ${component})
  install(DIRECTORY /opt/orin-openmp/licenses/
    DESTINATION ${CMAKE_INSTALL_DATADIR}/licenses/casbot_calib/openmp COMPONENT ${component})
endfunction()
