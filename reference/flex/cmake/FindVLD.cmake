# Check variable
if(NOT TARGET vld)
set(VLD_INSTALL_LOCATION "")
if(DEFINED ENV{VLD_INSTALL_DIR})
    set(VLD_INSTALL_LOCATION "$ENV{VLD_INSTALL_DIR}")
elseif(DEFINED VLD_INSTALL_DIR)
    set(VLD_INSTALL_LOCATION "${VLD_INSTALL_DIR}")
else()
    message(FATAL_ERROR "--- Microsoft VLD is required but variable \"VLD_INSTALL_DIR\" is not assigning any vld installation position!")
    return()
endif()

add_library(vld SHARED IMPORTED)

set(vld_IMPORTLIB_DIR "${VLD_INSTALL_LOCATION}")

set_target_properties(vld PROPERTIES
    IMPORTED_LOCATION                   "${vld_IMPORTLIB_DIR}/bin/Win64/vld_x64.dll"
    IMPORTED_IMPLIB                     "${vld_IMPORTLIB_DIR}/lib/Win64/vld.lib"
    INTERFACE_INCLUDE_DIRECTORIES       "${vld_IMPORTLIB_DIR}/include"
)

endif()