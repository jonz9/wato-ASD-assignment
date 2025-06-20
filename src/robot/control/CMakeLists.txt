cmake_minimum_required(VERSION 3.10)
project(control)

# Set compiler to use C++ 17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Search for dependencies required for building this package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(control_lib
  src/control_core.cpp)
# Indicate to compiler where to search for header files
target_include_directories(control_lib
  PUBLIC
  include)
# Add ROS2 dependencies required by package
ament_target_dependencies(control_lib
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)


# Create ROS2 node executable from source files
add_executable(control_node src/control_node.cpp)

target_include_directories(control_node
  PUBLIC
  include)
# Link to the previously built library to access Transformer classes and methods
target_link_libraries(control_node control_lib)

ament_target_dependencies(control_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

# Copy executable to installation location
install(TARGETS
  control_node
  control_lib
  DESTINATION lib/${PROJECT_NAME})

# Install headers
install(DIRECTORY include/
  DESTINATION include)
# Copy launch and config files to installation location
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})


ament_package()
