cmake_minimum_required(VERSION 3.8)
project(mobile_manip_task)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mobile_manip_interfaces REQUIRED)

add_executable(fake_vision_node src/fake_vision_node.cpp)

ament_target_dependencies(fake_vision_node
  rclcpp
  geometry_msgs
)

add_executable(robot_state_node src/robot_state_node.cpp)

ament_target_dependencies(robot_state_node
  rclcpp
  std_msgs
)

add_executable(task_executor_node src/task_executor_node.cpp)
  
ament_target_dependencies(task_executor_node
  rclcpp
  geometry_msgs
  std_msgs
  mobile_manip_interfaces
)

install(TARGETS
  fake_vision_node
  robot_state_node
  task_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
