PROJECT(youBot_KeyboardRemoteControl)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

## check required prerequisites
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/external/cmake_modules")
SET(Boost_USE_STATIC_LIBS ON)
SET(Boost_ADDITIONAL_VERSIONS "1.39" "1.39.0" "1.40" "1.40.0" "1.41" "1.41.0" "1.42" "1.42.0" "1.43" "1.43.0" "1.44" "1.44.0" "1.45" "1.45.0" "1.46" "1.46.0")
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread date_time filesystem system)

OPTION(USE_ROS "Enable ROS as compile tool" ON)

SET(ROS_ROOT_PATH $ENV{ROS_ROOT})
IF(DEFINED ROS_ROOT_PATH AND USE_ROS MATCHES ON)
	include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
    rosbuild_init()
    rosbuild_find_ros_package(youbot_driver)
    SET(OODL_YOUBOT_LIBRARIES YouBotDriver)
    SET(OODL_YOUBOT_CONFIG_DIR ${youbot_driver_PACKAGE_PATH}/config/)
ELSE(DEFINED ROS_ROOT_PATH AND USE_ROS MATCHES ON)
    FIND_PACKAGE(OODL_YouBot REQUIRED)
ENDIF(DEFINED ROS_ROOT_PATH AND USE_ROS MATCHES ON)

## preprocessor definitions
ADD_DEFINITIONS(-DYOUBOT_CONFIGURATIONS_DIR="${OODL_YOUBOT_CONFIG_DIR}/")
ADD_DEFINITIONS(-DCONFIG_DIR="${PROJECT_SOURCE_DIR}/config")

## build parameters and paths
SET(CMAKE_BUILD_TYPE DEBUG) #enable debug mode (e.g. for embedded gdb in eclipse )
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/ CACHE PATH "Configure the executable output path.")
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib/ CACHE PATH "Configure the library output path.")

## Set include directories
INCLUDE_DIRECTORIES(
	${OODL_YOUBOT_INCLUDE_DIR}
	${Boost_INCLUDE_DIR}
)

## Compile and link
ADD_EXECUTABLE(youBot_KeyboardRemoteControl
  src/main.cpp
)

TARGET_LINK_LIBRARIES(youBot_KeyboardRemoteControl
	${OODL_YOUBOT_LIBRARIES}
	${Boost_LIBRARIES}
	pthread
	ncurses
)

INSTALL(TARGETS youBot_KeyboardRemoteControl RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
