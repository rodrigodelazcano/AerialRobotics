#cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 3.0)

project(offboard)



## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  sensor_msgs
  std_msgs
  roslaunch
  message_generation
)

find_package(cmake_modules REQUIRED)


add_message_files(DIRECTORY msg
FILES
Cycle.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
DEPENDENCIES
std_msgs  # Or other packages containing msgs
)


roslaunch_add_file_check(launch)

find_library (GeographicLib_LIBRARIES Geographic
  PATHS "${CMAKE_INSTALL_PREFIX}/../GeographicLib/lib")

if (GeographicLib_LIBRARIES)
  get_filename_component (GeographicLib_LIBRARY_DIRS
    "${GeographicLib_LIBRARIES}" PATH)

  get_filename_component (_ROOT_DIR "${GeographicLib_LIBRARY_DIRS}" PATH)
  set (GeographicLib_INCLUDE_DIRS "${_ROOT_DIR}/../include")
  set (GeographicLib_BINARY_DIRS "${_ROOT_DIR}/../bin")

  # Required for Debian-alike systems
  find_library (GeographicLib_LIBRARIES_DEBIAN libGeographic.so
    PATHS "/usr/lib/x86_64-linux-gnu")
  if (GeographicLib_LIBRARIES_DEBIAN)
    set (GeographicLib_LIBRARIES ${GeographicLib_LIBRARIES_DEBIAN})
    get_filename_component (GeographicLib_LIBRARY_DIRS
      "${GeographicLib_LIBRARIES}" PATH)
  endif ()
  endif ()


catkin_package(
  INCLUDE_DIRS include

  CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime
)

install(
  DIRECTORY launch src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
include_directories(
  include
  ${roscpp_INCLUDE_DIRS}
  ${robag_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)



## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node 
  src/offboard.cpp
  src/takeoff.cpp
  src/sensors.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )

