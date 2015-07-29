cmake_minimum_required(VERSION 2.8.3)
project(move_gazebo_model)

find_package(catkin_simple REQUIRED)

catkin_simple()

# Executables
cs_add_executable(move_gazebo_model src/move_gazebo_model.cpp)

cs_install()
cs_export()
    
