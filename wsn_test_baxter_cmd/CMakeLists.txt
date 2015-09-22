cmake_minimum_required(VERSION 2.8.3)
project(wsn_test_baxter_cmd)

find_package(catkin REQUIRED COMPONENTS
  baxter_core_msgs
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
# add_executable(wsn_test_baxter_cmd_node src/wsn_test_baxter_cmd_node.cpp)
add_executable(wsn_test_baxter_cmd_node src/wsn_test_baxter_cmd.cpp)
add_executable(wsn_test_baxter_cmd2 src/wsn_test_baxter_cmd2.cpp)
add_executable(wsn_test_baxter_kbd src/wsn_test_baxter_kbd.cpp)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(wsn_test_baxter_cmd_node baxter_core_msgs_gencpp)
add_dependencies(wsn_test_baxter_cmd2 baxter_core_msgs_gencpp)

## Specify libraries to link a library or executable target against


target_link_libraries(wsn_test_baxter_cmd_node
  ${catkin_LIBRARIES}
)
target_link_libraries(wsn_test_baxter_cmd2
  ${catkin_LIBRARIES}
)
target_link_libraries(wsn_test_baxter_kbd
  ${catkin_LIBRARIES}
)