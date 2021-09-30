# Define RCLCPP_LOG_MIN_SEVERITY=RCLCPP_LOG_MIN_SEVERITY_[DEBUG|INFO|WARN|ERROR|FATAL] in your build options to compile out anything below that severity. Use RCLCPP_LOG_MIN_SEVERITY_NONE to compile out all macros.
add_compile_definitions(RCLCPP_LOG_MIN_SEVERITY=RCLCPP_LOG_MIN_SEVERITY_DEBUG)

cmake_minimum_required(VERSION 3.5)
project(mocap_optitrack_w2b)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Add the custom interfaces for the mocap system
find_package(rclcpp REQUIRED)
find_package(mocap_optitrack_interfaces REQUIRED)

#Include the headers.
#NOTE : libeiegen3-dev is assumed to be installed in the system
include_directories(include)

#Create the executables
add_executable(mocap_optitrack_w2b src/WorldToBase.cpp)
ament_target_dependencies(mocap_optitrack_w2b rclcpp mocap_optitrack_interfaces)

install(TARGETS 
  mocap_optitrack_w2b
  DESTINATION lib/${PROJECT_NAME})

#Add the configuration file to the binaries
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
