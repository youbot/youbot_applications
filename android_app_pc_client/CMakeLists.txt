PROJECT(youBot_Android)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(ROS_ROOT_PATH $ENV{ROS_ROOT})

IF(DEFINED ROS_ROOT_PATH)
	include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_find_ros_package(youbot_driver)
ENDIF(DEFINED ROS_ROOT_PATH)


## check required prerequisites
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/external/cmake_modules")
SET(Boost_USE_STATIC_LIBS ON)
SET(Boost_ADDITIONAL_VERSIONS "1.39" "1.39.0" "1.40" "1.40.0" "1.41" "1.41.0" "1.42" "1.42.0" "1.43" "1.43.0" "1.44" "1.44.0" "1.45" "1.45.0" "1.46" "1.46.0")
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread date_time filesystem system)
FIND_PACKAGE(Soem REQUIRED)
FIND_PACKAGE(OODL_YouBot REQUIRED)
FIND_PACKAGE(Bluetooth REQUIRED)

## preprocessor definitions
ADD_DEFINITIONS(-DYOUBOT_CONFIGURATIONS_DIR="${OODL_YOUBOT_CONFIG_DIR}")

## build parameters and paths
SET(CMAKE_BUILD_TYPE DEBUG) #enable debug mode (e.g. for embedded gdb in eclipse )
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/bin/ CACHE PATH "Configure the executable output path.")
SET(LIBRARY_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/lib/ CACHE PATH "Configure the library output path.")

## Set include directories
INCLUDE_DIRECTORIES(
	${OODL_YOUBOT_INCLUDE_DIR}
	${SOEM_INCLUDE_DIR}
	${Boost_INCLUDE_DIR}
	${BLUETOOTH_INCLUDE_DIR}
)

## Compile and link
ADD_EXECUTABLE(youBotAndroid
  src/main.cpp 
)

TARGET_LINK_LIBRARIES(youBotAndroid 
	${OODL_YOUBOT_LIBRARIES}
	${SOEM_LIBRARIES}
	${Boost_LIBRARIES}
	${BLUETOOTH_LIBRARIES}
	pthread
)

INSTALL(TARGETS youBotAndroid RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
