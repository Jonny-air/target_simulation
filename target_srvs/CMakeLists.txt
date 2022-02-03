cmake_minimum_required(VERSION 3.10.2)
project(target_srvs)

add_definitions(-std=c++17)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

##########
# EXPORT #
##########
cs_install()
cs_export()
