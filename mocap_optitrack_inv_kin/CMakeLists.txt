cmake_minimum_required(VERSION 3.5)
project(mocap_optitrack_inv_kin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Add the custom interfaces for the mocap system
find_package(rclcpp REQUIRED)#publisher
find_package(mocap_optitrack_interfaces REQUIRED)


ament_package()
