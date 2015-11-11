cmake_minimum_required(VERSION 2.8.3)
project(example_sensor_guided_motion)

find_package(catkin_simple REQUIRED)

catkin_simple()
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_library(my_lib src/my_lib.cpp)   

# Executables
cs_add_executable(example_sensor_guided_motion_client src/example_sensor_guided_motion_client.cpp)
# target_link_library(example my_lib)

cs_install()
cs_export()
    