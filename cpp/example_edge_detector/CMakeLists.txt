cmake_minimum_required(VERSION 3.8)
project(example_edge_detector)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[example_controller_plugin]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  cv_bridge
  OpenCV
  image_geometry
  image_transport
  mrs_lib
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  )

set(LIBRARIES
  ExampleEdgeDetector_EdgeDetector
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  include
  # you need to explicitly list packages from which you want to include
  ${mrs_lib_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## --------------------------------------------------------------
## |                          compiles                          |
## --------------------------------------------------------------

add_library(ExampleEdgeDetector_EdgeDetector SHARED
  src/edge_detector.cpp
  )

target_link_libraries(ExampleEdgeDetector_EdgeDetector
  ${OpenCV_LIBS}
)

ament_target_dependencies(ExampleEdgeDetector_EdgeDetector
  ${DEPENDENCIES}
)

target_compile_definitions(ExampleEdgeDetector_EdgeDetector PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

rclcpp_components_register_nodes(ExampleEdgeDetector_EdgeDetector PLUGIN "example_edge_detector::EdgeDetector" EXECUTABLE ExampleEdgeDetector_EdgeDetector)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
)

install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY tmux
  USE_SOURCE_PERMISSIONS
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
