cmake_minimum_required(VERSION 3.1.2)
project(vision_example)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  std_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  image_geometry
  mrs_lib
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES EdgeDetect
  CATKIN_DEPENDS roscpp mrs_lib image_transport std_msgs
)

include_directories(include
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(EdgeDetect src/EdgeDetect.cpp)

add_dependencies(EdgeDetect ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(EdgeDetect ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
