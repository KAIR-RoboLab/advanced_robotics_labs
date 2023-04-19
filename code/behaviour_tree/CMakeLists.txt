cmake_minimum_required(VERSION 3.16)
project(bt_lab)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_action
  nav2_msgs
  tf2
  behaviortree_cpp
  behaviortree_ros2
)

include_directories(include)


add_executable(bt_node src/bt_manager.cpp src/bt_node.cpp)

ament_target_dependencies(bt_node
  rclcpp
  rclcpp_action
  nav2_msgs
  tf2
  behaviortree_cpp
  behaviortree_ros2
)

install(TARGETS
  bt_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
