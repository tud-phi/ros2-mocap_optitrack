#!/bin/bash

#Build the package
colcon build --packages-select mocap_optitrack_pub

#Source the workspace 
. install/setup.bash

#Specify the directory of the compiled NatNet library
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/pietro/Scrivania/TESI/ROS2/MoCap/Ros2MoCap/src/mocap_pub/lib

#Run the node
ros2 run mocap_optitrack_pub mocap_optitrack_pub
