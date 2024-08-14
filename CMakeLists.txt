cmake_minimum_required(VERSION 3.5)
project(bag_recorder)

# C++ Standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Compiler options
if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Wpedantic")
endif()

# find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time regex program_options filesystem)
find_package(BZip2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Support large bags (>2GB) on 32-bit systems
add_definitions(-D_FILE_OFFSET_BITS=64)

# Add message files
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Rosbag.msg
  DEPENDENCIES std_msgs
)

include_directories(include 
  ${Boost_INCLUDE_DIRS}
  ${rosidl_default_generators_INCLUDE_DIRS}
)

#! this can replaced with ament_target_lib im sure
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rosbag2_cpp_INCLUDE_DIRS}
  ${BZip2_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${rosidl_default_generators_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)


message(STATUS "Include dirs: ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp")

add_library(recorder src/bag_recorder.cpp)
ament_target_dependencies(recorder
  rclcpp
  rosbag2_cpp
  Boost
)

# add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME})


# make executable
add_executable(${PROJECT_NAME}_node src/rosbag_recorder_node.cpp src/bag_launcher.cpp src/heartbeat.cpp)
target_link_libraries(${PROJECT_NAME}_node
  recorder
  ${Boost_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rosidl_default_runtime
)

link_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)


ament_export_dependencies(rosidl_default_runtime)

# Install executable and libraries
install(TARGETS ${PROJECT_NAME}_node recorder
  DESTINATION lib/${PROJECT_NAME})

# Install headers
install(DIRECTORY include/
  DESTINATION include)

# Install launch files and other directories if needed

ament_package()
