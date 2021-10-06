# Define RCLCPP_LOG_MIN_SEVERITY=RCLCPP_LOG_MIN_SEVERITY_[DEBUG|INFO|WARN|ERROR|FATAL] in your build options to compile out anything below that severity. Use RCLCPP_LOG_MIN_SEVERITY_NONE to compile out all macros.
add_compile_definitions(RCLCPP_LOG_MIN_SEVERITY=RCLCPP_LOG_MIN_SEVERITY_DEBUG)

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
find_package(rclcpp_components REQUIRED)
find_package(mocap_optitrack_interfaces REQUIRED)

#Download the NatNet SDK library
include(FetchContent)
fetchcontent_declare(natnet_sdk_content
  URL https://s3.amazonaws.com/naturalpoint/software/NatNetSDKLinux/ubuntu/NatNet_SDK_4.0_ubuntu.tar
)
fetchcontent_makeavailable(natnet_sdk_content)

add_library(natnet_sdk SHARED IMPORTED)
target_include_directories(natnet_sdk INTERFACE ${natnet_sdk_content_SOURCE_DIR}/include)
set_target_properties(natnet_sdk PROPERTIES IMPORTED_LOCATION ${natnet_sdk_content_SOURCE_DIR}/lib/libNatNet.so)


#Include the headers for the MoCap class
include_directories(include/MoCapPublisher)
#Include the headers for the NatNet client
include_directories(include/MoCapNatNetClient)

# TARGETS
add_executable(mocap_optitrack_client src/MoCapNatNetClient.cpp src/MoCapPublisher.cpp)
target_include_directories(mocap_optitrack_client
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${mocap_optitrack_interfaces_INCLUDE_DIRS}
)
target_link_libraries(mocap_optitrack_client
PUBLIC
  natnet_sdk
  rclcpp::rclcpp
  ${mocap_optitrack_interfaces_LIBRARIES}
)

# INSTALLATION
install(TARGETS 
  mocap_optitrack_client
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES                 ${natnet_sdk_content_SOURCE_DIR}/lib/libNatNet.so
  DESTINATION           lib
)

#Add the configuration file to the binaries
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()