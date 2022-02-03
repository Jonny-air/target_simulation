# Optional arguments to be passed into file
# ADDITIONAL_INCLUDE_DIRS           string  Additional include directories to add to every build target (PX4 uses this).
# BUILD_MAVLINK_INTERFACE_PLUGIN    bool    Build mavlink_interface_plugin (requires mav dependency).
# BUILD_OCTOMAP_PLUGIN              bool    Build the optical map plugin (requires Octomap).
# BUILD_OPTICAL_FLOW_PLUGIN         bool    Build the optical flow plugin (requires OpenCV).
# MAVLINK_HEADER_DIR                string  Location of MAVLink header files. If not provided, this CMakeLists.txt file will
#                                               search the default locations (e.g. ROS) for them. This variable is only required
#                                               if BUILD_MAVLINK_INTERFACE_PLUGIN=TRUE.

cmake_minimum_required(VERSION 2.8.3)
project(target_gazebo)

#
#if ( ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR} )
#  message( FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt." )
#endif()

#message(STATUS "rotors_gazebo_plugins CMakeLists.txt called.")

# =============================================================================================== #
# ========================== SET DEFAULTS FOR PASSED-IN VARIABLES =============================== #
# =============================================================================================== #

# Add any additional include directories as specified by the calling process (either user or another CMake file).
# ASL: Doesn't use this, catkin manages the mav_comm dependency
# PX4: Provides include directory for mav_msgs, so that "mav_msgs/default_topics.h" can be found and used.
# include_directories(${ADDITIONAL_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
message(STATUS "CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}")

# Initialise a list which will keep track of all targets
# that need to be installed.
set(targets_to_install "")

# =============================================================================================== #
# ======================================== STATUS MESSAGES ====================================== #
# =============================================================================================== #

message(STATUS "ADDITIONAL_INCLUDE_DIRS = ${ADDITIONAL_INCLUDE_DIRS}")

# Specify C++11 standard
add_definitions(-std=c++11)

# To enable assertions when compiled in release mode.
add_definitions(-DROS_ASSERT_ENABLED)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  geometry_msgs
  roscpp
  tf
)

message(STATUS "CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}" )

find_package(gazebo_dev QUIET)
if (NOT GAZEBO_DEV_FOUND)
  find_package(gazebo REQUIRED)
endif()

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# ============================================== #
# =================== PROTOBUF ================= #
# ============================================== #

# Not needed if used with the rotors simulator, as the messages are already built there
# if you need to build the messaged uncomment the section below

# set(PROTOBUF_IMPORT_DIRS "")
# foreach(ITR ${GAZEBO_INCLUDE_DIRS})
#   if(ITR MATCHES ".*gazebo-[0-9.]+$")
#     set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
#   endif()
# endforeach()
# message(STATUS "PROTOBUF_IMPORT_DIRS = " ${PROTOBUF_IMPORT_DIRS})

# # protobuf required for publishing/subscribing to Gazebo
# # messages
# # WARNING: THIS MUST BE RUN AFTER PROTOBUF_IMPORT_DIRS IS SETUP
# find_package(Protobuf REQUIRED)
# pkg_check_modules(PROTOBUF protobuf)

# # Protobuf version check
# if ("${PROTOBUF_VERSION}" VERSION_LESS "2.5.0")
#   message(FATAL_ERROR "protobuf version: ${PROTOBUF_VERSION} not compatible, must be >= 2.5.0")
# endif()

# # After running this block of code,
# # we should have something similar to
# # GAZEBO_MSG_INCLUDE_DIRS = /usr/include/gazebo-7/gazebo/msgs
# set(GAZEBO_MSG_INCLUDE_DIRS)
# foreach(ITR ${GAZEBO_INCLUDE_DIRS})
#   if(ITR MATCHES ".*gazebo-[0-9.]+$")
#     set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
#   endif()
# endforeach()

# # Get lists of all .proto files in the msgs directory
# file(GLOB msgs msgs/*.proto)

# # Finally, generate the .cpp files from the .proto files
# PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})

# # Create a shared library of protobuf messages (.so extension on Linux platforms)
# add_library(mav_msgs SHARED ${PROTO_SRCS})
# target_link_libraries(mav_msgs ${PROTOBUF_LIBRARY} gazebo_msgs)

# # This causes mav_msgs to be linked with every created library in this file from this
# # point forward.
# # NOTE: This is deprecated, should be using target_link_libraries instead
# link_libraries(mav_msgs)

# ============================================== #
# ==================== CATKIN ================== #
# ============================================== #

if(${gazebo_dev_FOUND})
  catkin_package(
    INCLUDE_DIRS include  
    CATKIN_DEPENDS geometry_msgs roscpp gazebo_plugins tf
  )
  set(target_linking_LIBRARIES ${catkin_LIBRARIES} ${gazebo_dev_LIBRARIES})
else()
  catkin_package(
    INCLUDE_DIRS include  
    CATKIN_DEPENDS geometry_msgs roscpp gazebo_plugins tf
  )
  set(target_linking_LIBRARIES ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
endif()

# Including GAZEBO_MSG_INCLUDE_DIRS here allows the .cpp files generated by custom .proto files to find
# the .cpp/hpp files generated by the built-in Gazebo .proto files
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_MSG_INCLUDE_DIRS})

# =============================================================================================== #
# ========================================= USER LIBRARIES ====================================== #
# =============================================================================================== #

#==================================== ROS POSITION PLUGIN ========================================//
add_library(rotors_gazebo_position_from_ros SHARED src/gazebo_position_from_ros.cpp)
target_link_libraries(rotors_gazebo_position_from_ros ${target_linking_LIBRARIES} )
add_dependencies(rotors_gazebo_position_from_ros ${catkin_EXPORTED_TARGETS})
list(APPEND targets_to_install rotors_gazebo_position_from_ros)

message(STATUS "CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}")
set(BIN_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
set(LIB_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Install all targets that were defined above (some conditionally) and added to the
# variable targets_to_install
install(
  TARGETS
    ${targets_to_install}
  DESTINATION ${BIN_DESTINATION}
  LIBRARY DESTINATION ${LIB_DESTINATION}
)

#message(FATAL_ERROR "Reached EOF.")
