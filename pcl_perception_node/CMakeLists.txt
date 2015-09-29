cmake_minimum_required(VERSION 2.8.3)
project(pcl_perception_node)

#find_package(catkin_simple REQUIRED)

#catkin_simple()
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

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

# WATCH OUT: C++11 compilation causes PCL-1.7 to segfault!!
# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

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


# Executables

add_executable(pcl_perception_node2 src/pcl_perception_node2.cpp)
add_executable(bed_of_nails_node src/bed_of_nails_node.cpp)

# target_link_library(example my_lib)
target_link_libraries(pcl_perception_node2 ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(bed_of_nails_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_dependencies(pcl_perception_node2 cwru_srv_gencpp)
add_dependencies(bed_of_nails_node cwru_srv_gencpp)