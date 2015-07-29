cmake_minimum_required(VERSION 2.8.3)
project(pcl_perception_node)

#find_package(catkin_simple REQUIRED)

#catkin_simple()
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cwru_srv
  pcl_ros
  pcl_conversions
  tf
)
add_definitions(${EIGEN_DEFINITIONS})
include_directories(
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ##${PROJECT_NAME}/include
  #${baxter_kinematics_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})


# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)

#WATCH OUT: C++11 compilation causes PCL-1.7 to segfault!!
# C++0x support - not quite the same as final C++11!
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

## Generate actions in the 'action' folder
#add_action_files(
#  FILES
#  cartTraj.action
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   actionlib_msgs trajectory_msgs std_msgs)

catkin_package(
  #INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME} 
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs  cwru_srv 
  DEPENDS eigen system_lib 
)

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
)

# Libraries
# cs_add_libraries(my_lib src/my_lib.cpp)   
# add_library(
#  ${PROJECT_NAME}  
  ##src/cwru_baxter/baxter_kinematics/src/baxter_kinematics.cpp
  ##${PROJECT_NAME}/src/baxter_kinematics.cpp
  ##${${PROJECT_NAME}_SRCS}
  # this library gets named after the package...cartesian_moves; find in devel/lib/libcartesian_moves.so
  ## src/${PROJECT_NAME}/baxter_kinematics.cpp
 #)
# Executables
add_executable(pcl_perception_node2 src/pcl_perception_node2.cpp)
add_executable(bed_of_nails_node src/bed_of_nails_node.cpp)
# target_link_library(example my_lib)
target_link_libraries(pcl_perception_node2 ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(bed_of_nails_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
cs_install()
cs_export()
    
