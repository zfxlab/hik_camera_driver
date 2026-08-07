# Find the Hikrobot MVS SDK without embedding proprietary SDK files in this package.
#
# Inputs:
#   HIK_MVS_ROOT             SDK root containing include/ and lib/
#   HIK_MVS_INSTALL_RUNTIME  Install the discovered .so files with this package
#
# Outputs:
#   HikMVS_FOUND
#   HIK_MVS_INCLUDE_DIR
#   HIK_MVS_LIBRARY_DIR
#   HikMVS::MvCameraControl

set(HIK_MVS_ROOT "" CACHE PATH "Root directory of the Hikrobot MVS SDK")
option(HIK_MVS_INSTALL_RUNTIME "Install discovered proprietary MVS shared libraries" ON)

set(_hik_mvs_roots
  "${HIK_MVS_ROOT}"
  "$ENV{HIK_MVS_ROOT}"
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/MVS"
  "/opt/MVS"
)

find_path(HIK_MVS_INCLUDE_DIR
  NAMES MvCameraControl.h
  HINTS ${_hik_mvs_roots}
  PATH_SUFFIXES include
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(_hik_mvs_lib_suffixes lib/aarch64 lib/arm64 lib/64 lib)
elseif(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(_hik_mvs_lib_suffixes lib/64 lib/amd64 lib/x86_64 lib)
else()
  set(_hik_mvs_lib_suffixes lib/32 lib/x86 lib)
endif()

find_library(HIK_MVS_CONTROL_LIBRARY
  NAMES MvCameraControl
  HINTS ${_hik_mvs_roots}
  PATH_SUFFIXES ${_hik_mvs_lib_suffixes}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(HikMVS
  REQUIRED_VARS HIK_MVS_INCLUDE_DIR HIK_MVS_CONTROL_LIBRARY
)

if(HikMVS_FOUND AND NOT TARGET HikMVS::MvCameraControl)
  get_filename_component(HIK_MVS_LIBRARY_DIR "${HIK_MVS_CONTROL_LIBRARY}" DIRECTORY)
  add_library(HikMVS::MvCameraControl SHARED IMPORTED)
  set_target_properties(HikMVS::MvCameraControl PROPERTIES
    IMPORTED_LOCATION "${HIK_MVS_CONTROL_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${HIK_MVS_INCLUDE_DIR}"
  )
endif()

mark_as_advanced(HIK_MVS_INCLUDE_DIR HIK_MVS_CONTROL_LIBRARY HIK_MVS_LIBRARY_DIR)
