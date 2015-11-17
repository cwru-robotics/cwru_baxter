cmake_minimum_required(VERSION 2.8.3)
project(baxter_gripper)

find_package(catkin_simple REQUIRED)

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_library(my_lib src/my_lib.cpp)   

# Executables
cs_add_executable(baxter_gripper src/baxter_gripper.cpp src/ReadWrite.c src/dynamixel.c src/dxl_hal.c)
#cs_add_executable(dynamixel_motor2_node src/dynamixel_motor2_node.cpp src/ReadWrite.c src/dynamixel.c src/dxl_hal.c)
cs_add_executable(dynamixel_motor_node src/dynamixel_motor_node.cpp src/ReadWrite.c src/dynamixel.c src/dxl_hal.c)
cs_add_executable(dynamixel_sin_test src/dynamixel_sin_test.cpp )
# target_link_library(example my_lib)

cs_install()
cs_export()
    
