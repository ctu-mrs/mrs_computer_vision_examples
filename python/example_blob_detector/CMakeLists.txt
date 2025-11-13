cmake_minimum_required(VERSION 3.5)
project(example_blob_detector)

set(DEPENDENCIES
  ament_cmake
  ament_cmake_python
  rclpy
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  )

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  )

file(GLOB PYTHON_ENV_FILES python-env/*)
install(FILES ${PYTHON_ENV_FILES} DESTINATION share/${PROJECT_NAME}/python-env)

install(
  DIRECTORY ${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  DESTINATION lib
  )

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY tmux
  USE_SOURCE_PERMISSIONS
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()
