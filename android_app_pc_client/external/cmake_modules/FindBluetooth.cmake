# - Try to find BlueZ development library
# Once done this will define
#
#  BLUETOOTH_FOUND - bluetooth found
#  BLUETOOTH_INCLUDE_DIR - the bluetooth include directory
#  BLUETOOTH_LIBRARIES - bluetooth library


FIND_PATH(BLUETOOTH_INCLUDE_DIR NAMES bluetooth.h
  PATHS
  $ENV{YOUBOTDIR}/include/bluetooth/
  $ENV{ROBOTPKG_BASE}/include/bluetooth/
  ENV CPATH
  /usr/include/
  /usr/include/bluetooth/
  /usr/local/include/
  /usr/local/include/bluetooth/
  /opt/local/include/
  /opt/local/include/bluetooth/
  NO_DEFAULT_PATH
)

FIND_LIBRARY(BLUETOOTH_LIBRARIES NAMES "bluetooth"
  PATHS
  $ENV{YOUBOTDIR}/lib 
  $ENV{ROBOTPKG_BASE}/lib
  ENV LD_LIBRARY_PATH
  ENV LIBRARY_PATH
  /usr/lib
  /usr/local/lib
  /opt/local/lib
  NO_DEFAULT_PATH
)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set BLUETOOTH_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(bluetooth  DEFAULT_MSG
                                  BLUETOOTH_INCLUDE_DIR BLUETOOTH_LIBRARIES)

# show the BLUETOOTH_INCLUDE_DIR  and BLUETOOTH_LIBRARIES variables only in the advanced view
IF (BLUETOOTH_FOUND)
  MARK_AS_ADVANCED(BLUETOOTH_INCLUDE_DIR BLUETOOTH_LIBRARIES)
ENDIF (BLUETOOTH_FOUND)


