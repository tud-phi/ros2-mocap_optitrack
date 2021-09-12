cmake_minimum_required(VERSION 3.5)
project(mocap_optitrack_client)

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
find_package(rclcpp REQUIRED)#publisher
find_package(mocap_optitrack_interfaces REQUIRED)

#Include the headers for the MoCap class
include_directories(include/MoCapPublisher)
#Include the headers for the NatNet SDK
include_directories(include/NatNet)
#Include the headers for the NatNet client
include_directories(include/MoCapNatNetClient)

#Import the interfaces for the custom messages

#Add the directory where the libraries are located
link_directories(${CMAKE_SOURCE_DIR}/lib)


#Create the executables
add_executable(mocap_optitrack_client src/MoCapPublisher.cpp src/MoCapNatNetClient.cpp)
ament_target_dependencies(mocap_optitrack_client rclcpp mocap_optitrack_interfaces)


#Add all the used libraries to the project
target_link_libraries(mocap_optitrack_client NatNet)#in the folder ./lib there must be a file libNatNet.so
install(TARGETS 
  mocap_optitrack_client
  DESTINATION lib/${PROJECT_NAME})

#Add the configuration file to the binaries
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()