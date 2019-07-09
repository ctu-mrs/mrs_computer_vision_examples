cmake_minimum_required(VERSION 2.8.3)
project(vision_example)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  std_msgs
  cv_bridge
  mrs_lib
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES EdgeDetect
  CATKIN_DEPENDS roscpp mrs_lib image_transport std_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(EdgeDetect src/EdgeDetect.cpp)

add_dependencies(EdgeDetect ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(EdgeDetect ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
