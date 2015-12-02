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

cs_add_executable(dynamixel_motor_node src/dynamixel_motor_node.cpp src/dxl_c_files/ReadWrite.c src/dxl_c_files/dynamixel.c src/dxl_c_files/dxl_hal.c)

cs_add_executable(baxter_gripper_test src/baxter_gripper_test.cpp )
cs_add_executable(baxter_compliant_gripper src/baxter_compliant_gripper.cpp )



cs_install()
cs_export()
    
