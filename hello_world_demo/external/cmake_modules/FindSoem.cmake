# - Try to find soem
# Once done this will define
#
#  SOEM_FOUND - soem found
#  SOEM_INCLUDE_DIR - the soem include directory
#  SOEM_LIBRARY_DIR - soem lib directory
#

SET(SOEMLIB "soem")

FIND_PATH(SOEM_INCLUDE_DIR NAMES ethercatmain.h
  PATHS
  $ENV{ROBOTPKG_BASE}/include/soem/src 
  ${SOEM_PATH}/include/soem/src
	${youbot_driver_PACKAGE_PATH}/soem/src
  ENV CPATH
  /usr/include/
  /usr/include/soem/src
  /usr/local/include/
  /usr/local/include/soem/src
  /opt/local/include/
  /opt/local/include/soem/src
  NO_DEFAULT_PATH
)

#MARK_AS_ADVANCED("SOEM_INCLUDE_DIR: "${SOEM_INCLUDE_DIR})

FIND_LIBRARY(SOEM_LIBRARIES NAMES ${SOEMLIB} "SoemLibraries"
  PATHS
  $ENV{ROBOTPKG_BASE}/lib
  ${SOEM_PATH}/lib
	${youbot_driver_PACKAGE_PATH}/lib
  ENV LD_LIBRARY_PATH
  ENV LIBRARY_PATH
  /usr/lib
  /usr/local/lib
  /opt/local/lib
  NO_DEFAULT_PATH
)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set SOEM_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(SOEM  DEFAULT_MSG
                                  SOEM_INCLUDE_DIR SOEM_LIBRARIES)

IF(SOEM_FOUND)
  MARK_AS_ADVANCED(SOEM_INCLUDE_DIR SOEM_LIBRARIES)
ENDIF(SOEM_FOUND)


