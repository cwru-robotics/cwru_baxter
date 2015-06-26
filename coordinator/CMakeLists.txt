cmake_minimum_required(VERSION 2.8.3)
project(coordinator)

find_package(catkin_simple REQUIRED)
#need the following to use Eigen--careful of case sensitivity and version 3
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})
catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_libraries(my_lib src/my_lib.cpp)   

# Executables
cs_add_executable(coordinator src/coordinator.cpp)
# target_link_library(example my_lib)

cs_install()
cs_export()
    