cmake_minimum_required(VERSION 2.8.3)
project(cartesian_moves)

#find_package(catkin_simple REQUIRED)

#catkin_simple()
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(actionlib REQUIRED)
find_package(catkin REQUIRED COMPONENTS actionlib_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  baxter_core_msgs
  roscpp
  sensor_msgs
  std_msgs
  trajectory_msgs
  baxter_kinematics
  baxter_traj_streamer
  cwru_joint_space_planner
  cwru_srv
  actionlib_msgs
  visualization_msgs  
)

include_directories(
  ${Eigen_INCLUDE_DIRS}
  ${PROJECT_NAME}/include
  #${baxter_kinematics_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})


# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

## Generate actions in the 'action' folder
add_action_files(
  FILES
  cartTraj.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs trajectory_msgs std_msgs)

catkin_package(
  #INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME} cwru_joint_space_planner actionlib
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs baxter_core_msgs trajectory_msgs cwru_joint_space_planner baxter_kinematics cwru_srv   actionlib_msgs 
  DEPENDS eigen system_lib actionlib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Libraries
# cs_add_libraries(my_lib src/my_lib.cpp)   
 add_library(
  ${PROJECT_NAME}  
  ##src/cwru_baxter/baxter_kinematics/src/baxter_kinematics.cpp
  ##${PROJECT_NAME}/src/baxter_kinematics.cpp
  ##${${PROJECT_NAME}_SRCS}
  # this library gets named after the package...cartesian_moves; find in devel/lib/libcartesian_moves.so
  src/cart_path_planner_lib.cpp
  ## src/${PROJECT_NAME}/baxter_kinematics.cpp
 )
# Executables
#cs_add_executable(goto_prepose src/traj_action_client_pre_pose.cpp)
add_executable(goto_prepose src/traj_action_client_pre_pose.cpp)
add_executable(cart_traj_as src/cart_move_hand_down_as.cpp)
add_executable(cart_traj_ac src/cart_traj_action_client.cpp)
add_executable(test_cart_plan_lib src/test_cart_path_planner_lib.cpp)
# target_link_library(example my_lib)
target_link_libraries(goto_prepose ${catkin_LIBRARIES})
target_link_libraries(cart_traj_as ${catkin_LIBRARIES})
target_link_libraries(cart_traj_ac ${catkin_LIBRARIES})
target_link_libraries(test_cart_plan_lib cartesian_moves ${catkin_LIBRARIES})
cs_install()
cs_export()
    
