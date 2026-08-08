cmake_minimum_required(VERSION 3.16)

project(cooperative_localization_gz LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
add_subdirectory(
  "${REPO_ROOT}"
  "${CMAKE_CURRENT_BINARY_DIR}/adaptive_localization_core"
  EXCLUDE_FROM_ALL
)

add_executable(hidden_target_experiment_node
  src/hidden_target_experiment_node.cpp
)
target_link_libraries(hidden_target_experiment_node
  adaptive_localization_core
)
ament_target_dependencies(hidden_target_experiment_node
  rclcpp
  visualization_msgs
)

add_executable(hidden_target_batch_runner
  src/hidden_target_batch_runner.cpp
)
target_link_libraries(hidden_target_batch_runner
  adaptive_localization_core
)
ament_target_dependencies(hidden_target_batch_runner
  rclcpp
)

add_executable(measurement_emulator_node
  src/measurement_emulator_node.cpp
)
target_link_libraries(measurement_emulator_node
  adaptive_localization_core
)
ament_target_dependencies(measurement_emulator_node
  geometry_msgs
  rclcpp
  visualization_msgs
)

add_executable(supervised_closed_loop_node
  src/supervised_closed_loop_node.cpp
)
target_link_libraries(supervised_closed_loop_node
  adaptive_localization_core
)
ament_target_dependencies(supervised_closed_loop_node
  geometry_msgs
  nav_msgs
  rclcpp
  ros_gz_interfaces
)

install(TARGETS
  hidden_target_experiment_node
  hidden_target_batch_runner
  measurement_emulator_node
  supervised_closed_loop_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
