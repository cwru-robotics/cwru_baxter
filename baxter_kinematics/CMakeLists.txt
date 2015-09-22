cmake_minimum_required(VERSION 2.8.3)
project(baxter_kinematics)

find_package(cmake_modules REQUIRED)

find_package(Eigen REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  tf
  cwru_srv
)

include_directories(
  ${Eigen_INCLUDE_DIRS}
  ${PROJECT_NAME}/include
)

add_definitions(${EIGEN_DEFINITIONS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs tf baxter_core_msgs cwru_srv
  DEPENDS eigen system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  src/baxter_kinematics.cpp
 )

## Declare a cpp executable
# add_executable(baxter_kinematics_node src/baxter_kinematics_node.cpp)

add_executable(baxter_kinematics_test_main src/baxter_kinematics_test_main.cpp)
add_executable(baxter_kinematics_test_main2 src/baxter_kinematics_test_main2.cpp)

add_executable(baxter_kinematics_test_main3 src/baxter_kinematics_test_main3.cpp)
add_executable(baxter_kinematics_test_main4 src/baxter_kinematics_test_main4.cpp)
add_executable(baxter_kinematics_test_jacobian_main src/baxter_kinematics_test_jacobian_main.cpp)

target_link_libraries(baxter_kinematics_test_main baxter_kinematics ${catkin_LIBRARIES})
target_link_libraries(baxter_kinematics_test_main2 baxter_kinematics ${catkin_LIBRARIES})
target_link_libraries(baxter_kinematics_test_main3 baxter_kinematics ${catkin_LIBRARIES})
target_link_libraries(baxter_kinematics_test_main4 baxter_kinematics ${catkin_LIBRARIES})
target_link_libraries(baxter_kinematics_test_jacobian_main baxter_kinematics ${catkin_LIBRARIES})

add_dependencies(baxter_kinematics cwru_srv_gencpp)