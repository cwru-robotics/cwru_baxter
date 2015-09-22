cmake_minimum_required(VERSION 2.8.3)
project(move_gazebo_model)

find_package(catkin_simple REQUIRED)

catkin_simple()

# Executables
cs_add_executable(move_gazebo_model src/move_gazebo_model.cpp)
cs_add_executable(move_gazebo_model_interactive src/move_gazebo_model_interactive.cpp)
cs_add_executable(move_calibration_checkerboard src/move_calibration_checkerboard.cpp)
cs_install()
cs_export()
    
