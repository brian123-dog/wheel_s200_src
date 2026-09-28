# - Try to find serial library
# Once done this will define
#  serial_FOUND - System has serial
#  serial_INCLUDE_DIRS - The serial include directories
#  serial_LIBRARIES - The libraries needed to use serial

find_path(serial_INCLUDE_DIR
  NAMES serial/serial.h
  PATHS /usr/local/include /usr/include
)

find_library(serial_LIBRARY
  NAMES serial
  PATHS /usr/local/lib /usr/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(serial
  REQUIRED_VARS serial_LIBRARY serial_INCLUDE_DIR
)

if(serial_FOUND)
  set(serial_LIBRARIES ${serial_LIBRARY})
  set(serial_INCLUDE_DIRS ${serial_INCLUDE_DIR})
  if(NOT TARGET serial::serial)
    add_library(serial::serial UNKNOWN IMPORTED)
    set_target_properties(serial::serial PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${serial_INCLUDE_DIR}"
      IMPORTED_LOCATION "${serial_LIBRARY}"
    )
  endif()
endif()

mark_as_advanced(serial_INCLUDE_DIR serial_LIBRARY)

