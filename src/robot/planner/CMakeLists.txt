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
find_package(nav_msgs REQUIRED)    
find_package(geometry_msgs REQUIRED)     

add_library(planner_lib
  src/planner_core.cpp)
# Indicate to compiler where to search for header files
target_include_directories(planner_lib
  PUBLIC include)
# Add ROS2 dependencies required by package
ament_target_dependencies(planner_lib rclcpp nav_msgs geometry_msgs)

# Create ROS2 node executable from source files
add_executable(planner_node src/planner_node.cpp)
# Link to the previously built library to access planner classes and methods
target_link_libraries(planner_node planner_lib)

# Copy executable to installation location
install(TARGETS
  planner_node
  DESTINATION lib/${PROJECT_NAME})

# Copy launch and config files to installation location
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()