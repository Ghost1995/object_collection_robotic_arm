cmake_minimum_required(VERSION 2.8.3)
project(object_collection_robotic_arm)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# # Add project cmake modules to path.
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

# # We probably don't want this to run on every build.
# option(COVERAGE "Generate Coverage Data" ON)

# if (COVERAGE)
#     include(CodeCoverage)
#     setup_target_for_coverage(code_coverage ../devel/lib/object_collection_robotic_arm/allTests coverage)
#     set(COVERAGE_SRCS app/main.cpp include/Detection.hpp include/KukaKinematics.hpp include/KukaGripper.hpp)

#     SET(CMAKE_CXX_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
#     SET(CMAKE_C_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
#     SET(CMAKE_EXE_LINKER_FLAGS "-fprofile-arcs -ftest-coverage ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
# else()
#     set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic -g ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
# endif()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs cv_bridge image_transport)
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)

# include(CMakeToolsHelpers OPTIONAL)
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_CXX_STANDARD 14)

## The catkin_package macro generates cmake config files for your package
catkin_package(INCLUDE_DIRS include/object_collection_robotic_arm CATKIN_DEPENDS roscpp std_msgs)

## Build
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

## Specify additional locations of header files
include_directories(include/object_collection_robotic_arm ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(objSeg src/main.cpp src/KukaKinematics.cpp src/KukaGripper.cpp src/Detection.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(objSeg ${OpenCV_LIBS} ${catkin_LIBRARIES})

## Testing
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(gtest)

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")

  add_rostest_gtest(allTests test/allTests.test test/main.cpp test/testDetection.cpp test/testKukaKinematics.cpp test/testKukaGripper.cpp src/Detection.cpp src/KukaKinematics.cpp src/KukaGripper.cpp)
  target_link_libraries(allTests ${OpenCV_LIBS} ${catkin_LIBRARIES})
  add_dependencies(allTests ${catkin_EXPORTED_TARGETS})
endif()

