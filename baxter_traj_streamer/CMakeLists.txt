cmake_minimum_required(VERSION 2.8.3)
project(baxter_traj_streamer)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(actionlib REQUIRED)
find_package(catkin REQUIRED COMPONENTS actionlib_msgs)

find_package(catkin REQUIRED COMPONENTS
  baxter_core_msgs
  roscpp
  sensor_msgs
  std_msgs
  trajectory_msgs
  cwru_srv
  cwru_action
  actionlib_msgs  
)

include_directories(
  ${Eigen_INCLUDE_DIRS}
  ${PROJECT_NAME}/include
)

add_definitions(${EIGEN_DEFINITIONS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Generate actions in the 'action' folder
#add_action_files(
#  FILES
#  traj.action
#)


## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   actionlib_msgs trajectory_msgs std_msgs cwru_srv)


catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME}  actionlib
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs baxter_core_msgs trajectory_msgs cwru_srv cwru_action  actionlib_msgs 
  DEPENDS eigen system_lib actionlib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(baxter_traj_streamer
#   src/${PROJECT_NAME}/baxter_traj_streamer.cpp
# )
 add_library(
  ${PROJECT_NAME}  
  src/baxter_traj_streamer.cpp
 )
## Declare a cpp executable
# add_executable(baxter_traj_streamer_node src/baxter_traj_streamer_node.cpp)
#add_executable(baxter_traj_streamer_test_main src/baxter_traj_streamer_test_main.cpp)
#add_executable(baxter_traj_interpolator_node src/baxter_traj_interpolator_node.cpp)
#add_executable(elbow_orbit_plan_main src/elbow_orbit_plan_main.cpp)
#add_executable(elbow_orbit_plan_main2 src/elbow_orbit_plan_main2.cpp)
#add_executable(test_init_cmd src/test_init_cmd_main.cpp)
#add_executable(traj_action_client src/traj_action_client.cpp)

add_executable(traj_interpolator_as src/traj_interpolator_as.cpp)
add_executable(traj_action_client_pre_pose src/traj_action_client_pre_pose.cpp)
add_executable(get_and_save_r_arm_pose src/get_and_save_r_arm_pose.cpp)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(traj_action_client_pre_pose)
add_dependencies(traj_interpolator_as baxter_traj_streamer_generate_messages_cpp)
## Specify libraries to link a library or executable target against
# target_link_libraries(baxter_traj_streamer_node
#   ${catkin_LIBRARIES}
# )

#list baxter_traj_streamer explicitly, because this library is in the current package
#target_link_libraries(baxter_traj_streamer_test_main baxter_traj_streamer ${catkin_LIBRARIES})
#target_link_libraries(baxter_traj_interpolator_node  ${catkin_LIBRARIES})
#target_link_libraries(elbow_orbit_plan_main baxter_traj_streamer  ${catkin_LIBRARIES})
#target_link_libraries(elbow_orbit_plan_main2 baxter_traj_streamer  ${catkin_LIBRARIES})
#target_link_libraries(test_init_cmd baxter_traj_streamer  ${catkin_LIBRARIES})
#target_link_libraries(traj_action_client  ${catkin_LIBRARIES})

target_link_libraries(traj_interpolator_as baxter_traj_streamer  ${catkin_LIBRARIES})
target_link_libraries(traj_action_client_pre_pose baxter_traj_streamer ${catkin_LIBRARIES})
target_link_libraries(get_and_save_r_arm_pose baxter_traj_streamer  ${catkin_LIBRARIES})

add_dependencies(baxter_traj_streamer baxter_core_msgs_gencpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS baxter_traj_streamer baxter_traj_streamer_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_baxter_traj_streamer.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
