cmake_minimum_required(VERSION 3.10.2)
project(target_drone)

find_package(catkin_simple REQUIRED COMPONENTS)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_executable(target_drone
  src/target_drone_node.cc
  src/target_drone.cc)

cs_install()
cs_export()
