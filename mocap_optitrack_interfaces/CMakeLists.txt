cmake_minimum_required(VERSION 3.5)
project(mocap_optitrack_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# add the dependencies to create the custom interfaces
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RigidBody.msg"
  "msg/RigidBodyArray.msg"
  "msg/Configuration.msg"
  "msg/ConfigurationArray.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
 )


ament_package()
