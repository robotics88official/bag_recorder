cmake_minimum_required(VERSION 3.5)
project(bag_recorder)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Compiler options
if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Wpedantic")
endif()

include_directories(/opt/ros/humble/include)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(BZip2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rosbag2_compression REQUIRED)
find_package(rosbag2_storage REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
  ${rosbag2_cpp_INCLUDE_DIRS}
  ${BZip2_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${rosidl_default_generators_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# Declare ROS interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Rosbag.msg
  DEPENDENCIES std_msgs
)

# Add library
add_library(recorder src/bag_recorder.cpp)

# Add executable
add_executable(${PROJECT_NAME}_node src/rosbag_recorder_node.cpp src/bag_launcher.cpp src/heartbeat.cpp)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rcl_interfaces
  rosbag2_cpp
  rosbag2_compression
  rosbag2_storage
  std_msgs
  rosidl_default_runtime
  rosidl_typesupport_cpp
)

target_link_libraries(${PROJECT_NAME}_node  recorder ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_node 
  recorder
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY include/
#   DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
# Package setup
ament_package()
