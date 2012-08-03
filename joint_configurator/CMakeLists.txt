PROJECT(youBot_JointConfigurator)
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
SET(CMAKE_C_FLAGS_DEBUG "-fPIC -Wall -g")
SET(CMAKE_CXX_FLAGS_DEBUG "-fPIC -Wall -g")

IF(NOT DEFINED JOINT_CONFIGURATOR_CONFIG_DIR)
  SET(JOINT_CONFIGURATOR_CONFIG_DIR ${PROJECT_SOURCE_DIR}/config/)
ENDIF(NOT DEFINED JOINT_CONFIGURATOR_CONFIG_DIR)

ADD_DEFINITIONS(-DYOUBOT_CONFIGURATIONS_DIR="${OODL_YOUBOT_CONFIG_DIR}")
ADD_DEFINITIONS(-DCONFIG_DIR="${JOINT_CONFIGURATOR_CONFIG_DIR}")

## build parameters and paths
SET(CMAKE_BUILD_TYPE DEBUG) #enable debug mode (e.g. for embedded gdb in eclipse )
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin CACHE PATH "Configure the executable output path.")
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib CACHE PATH "Configure the library output path.")

## Set include directories
INCLUDE_DIRECTORIES(
	${OODL_YOUBOT_INCLUDE_DIR}
	${Boost_INCLUDE_DIR}
)

## Compile and link
ADD_EXECUTABLE(youBot_JointConfigurator
  src/JointConfiguratorMain.cpp
)

ADD_EXECUTABLE(youBot_ArmConfigurator
  src/ArmConfigurator.cpp
)

ADD_EXECUTABLE(youBot_BaseConfigurator
  src/BaseConfigurator.cpp
)

ADD_LIBRARY(JointConfigurator
  src/JointConfigurator.cpp
)

#ADD_LIBRARY(JointConfiguratorShared SHARED)
#SET_TARGET_PROPERTIES(JointConfiguratorShared PROPERTIES OUTPUT_NAME "JointConfigurator")
#TARGET_LINK_LIBRARIES(JointConfiguratorShared -Wl,-whole-archive JointConfigurator -Wl,-no-whole-archive)

SET(YOUBOT_DRIVER_CONFIG
${PROJECT_SOURCE_DIR}/config/arm-1.cfg
${PROJECT_SOURCE_DIR}/config/arm-2.cfg
${PROJECT_SOURCE_DIR}/config/arm-3.cfg
${PROJECT_SOURCE_DIR}/config/arm-4.cfg
${PROJECT_SOURCE_DIR}/config/arm-5.cfg
${PROJECT_SOURCE_DIR}/config/arm-1-protected.cfg
${PROJECT_SOURCE_DIR}/config/arm-2-protected.cfg
${PROJECT_SOURCE_DIR}/config/arm-3-protected.cfg
${PROJECT_SOURCE_DIR}/config/arm-4-protected.cfg
${PROJECT_SOURCE_DIR}/config/arm-5-protected.cfg
${PROJECT_SOURCE_DIR}/config/base.cfg
${PROJECT_SOURCE_DIR}/config/base-protected.cfg
)


TARGET_LINK_LIBRARIES(youBot_JointConfigurator
    ${OODL_YOUBOT_LIBRARIES}
	${Boost_LIBRARIES}
	pthread
    JointConfigurator
)

TARGET_LINK_LIBRARIES(youBot_ArmConfigurator
	${OODL_YOUBOT_LIBRARIES}
	${Boost_LIBRARIES}
	pthread
    JointConfigurator
)

TARGET_LINK_LIBRARIES(youBot_BaseConfigurator
	${OODL_YOUBOT_LIBRARIES}
	${Boost_LIBRARIES}
	pthread
    JointConfigurator
)

MACRO(INSTALL_CONFIG_WITH_DIRECTORY CONFIG_LIST)
  FOREACH(CONFIG ${${CONFIG_LIST}})
    STRING(REGEX MATCH "(.*)[/\\]" DIR ${CONFIG})
    INSTALL(FILES ${CONFIG} DESTINATION config)
  ENDFOREACH(CONFIG)
ENDMACRO(INSTALL_CONFIG_WITH_DIRECTORY)


INSTALL_CONFIG_WITH_DIRECTORY(YOUBOT_DRIVER_CONFIG)

INSTALL(TARGETS youBot_JointConfigurator youBot_ArmConfigurator RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
