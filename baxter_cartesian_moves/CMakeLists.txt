cmake_minimum_required(VERSION 2.8.3)
project(baxter_cartesian_moves)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()
# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)

# Libraries
cs_add_library(cart_path_planner_lib src/cart_path_planner_lib.cpp)  

cs_add_executable(baxter_cart_move_as src/baxter_cart_move_as.cpp)
cs_add_executable(example_baxter_cart_move_action_client src/example_baxter_cart_move_action_client.cpp)
target_link_libraries(baxter_cart_move_as cart_path_planner_lib ${catkin_LIBRARIES})
