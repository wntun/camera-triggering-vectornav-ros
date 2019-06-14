cmake_minimum_required(VERSION 2.8.3)
project(vectornav)
add_definitions(-std=c++11)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp std_msgs message_generation)

#find_package(Gphoto2 REQUIRED)

find_library(WIRINGPI_LIBRARY wiringPi /usr/include/)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

# Generate messages in the 'msg' folder
 add_message_files(
   FILES
   ku_gps.msg
#   ins.msg
#   gps.msg
#   sensors.msg
#   utc_time.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
DEPENDENCIES
  std_msgs 
#  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vectornav
   CATKIN_DEPENDS roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

#include(libphoto2/CMakeLists.txt)
#include(libgphoto2/CMakeLists.txt)
add_subdirectory(vnproglib-1.1/cpp)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(vnproglib-1.1/cpp/include 
	${Gphoto2_INCLUDE_DIRS}
	${WIRINGPI_LIBRARY}
	#libgphoto2/include 
	${catkin_INCLUDE_DIRS})
link_directories(vnproglib-1.1/cpp/build/bin 
	/usr/include/
	#libgphoto2/build/bin
	)

## Declare a cpp library
## Declare a cpp executable
add_executable(vnpub src/main.cpp)

#target_include_directories(${Gphoto2_INCLUDE_DIRS})

## Specify libraries to link a library or executable target against
target_link_libraries(vnpub
  vncxx
  ${Gphoto2_LIBRARIES}
  ${WIRINGPI_LIBRARY}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executable scripts (Python etc.) for installation

#install(PROGRAMS
#  scripts/gpsFix.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables and/or libraries for installation
install(TARGETS vnpub
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
