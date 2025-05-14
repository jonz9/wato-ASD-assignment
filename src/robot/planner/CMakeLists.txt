cmake_minimum_required(VERSION 3.10)
project(planner)

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Search for dependencies required for building this package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)       # ⬅ required for nav_msgs::msg
find_package(geometry_msgs REQUIRED)  # ⬅ required for geometry_msgs::msg

# Build library from core planner logic
add_library(planner_lib
  src/planner_core.cpp)
target_include_directories(planner_lib
  PUBLIC include)

ament_target_dependencies(planner_lib
  rclcpp
  nav_msgs
  geometry_msgs
)

# Build the ROS2 node executable
add_executable(planner_node src/planner_node.cpp)
target_link_libraries(planner_node planner_lib)
target_include_directories(planner_node
  PUBLIC include)
ament_target_dependencies(planner_node
  rclcpp
  nav_msgs
  geometry_msgs
)

# Install binaries
install(TARGETS
  planner_node
  DESTINATION lib/${PROJECT_NAME})

# Install configuration files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
