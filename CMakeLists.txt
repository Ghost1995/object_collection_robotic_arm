cmake_minimum_required(VERSION 2.8.3)
project(kuka_arm)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  geometry_msgs
  geometry_msgs
  roscpp
  roscpp
  roscpp
  rospy
  rospy
  rospy
  std_msgs
  std_msgs
  std_msgs
  tf
  iiwa_msgs
)
find_package( Eigen3 REQUIRED )
find_package( orocos_kdl REQUIRED )
add_compile_options(-std=c++11)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kuka_arm
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
FIND_PATH(EIGEN3_INCLUDE_DIR Eigen/Core /usr/include /usr/include/eigen3 /usr/local/include /usr/local/include/eigen3)
IF ( EIGEN3_INCLUDE_DIR )
    MESSAGE(STATUS "-- Looking for Eigen3 - found")
    SET(KDL_CFLAGS "${KDL_CFLAGS} -I${EIGEN3_INCLUDE_DIR}" CACHE INTERNAL "")
ELSE ( EIGEN3_INCLUDE_DIR )
    MESSAGE(FATAL_ERROR "-- Looking for Eigen3 - not found")
ENDIF ( EIGEN3_INCLUDE_DIR )
include_directories(
# include
  ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
)

add_executable(detect src/kuka_execute.cpp)
target_link_libraries(detect ${catkin_LIBRARIES})
