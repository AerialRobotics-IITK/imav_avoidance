cmake_minimum_required(VERSION 2.8.3)
project(avoidance)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIRS})

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/avoidance.cpp
)

############
# BINARIES #
############
cs_add_executable(avoidance_node src/avoidance_node.cpp)
target_link_libraries(avoidance_node  ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()