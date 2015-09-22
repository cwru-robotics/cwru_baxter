cmake_minimum_required(VERSION 2.8.3)
project(example_opencv)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  cwru_srv
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(image_converter src/image_converter.cpp)

add_executable(snapshot src/snapshot.cpp)
add_dependencies(snapshot cwru_srv_gencpp)

add_executable(images_timesync src/images_timesync.cpp)

target_link_libraries(image_converter ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
 )

target_link_libraries(snapshot ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
 )

target_link_libraries(images_timesync ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
 )