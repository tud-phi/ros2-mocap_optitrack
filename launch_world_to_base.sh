#!/bin/bash

#Build the package
colcon build --packages-select mocap_optitrack_w2b

#Source the workspace 
. install/setup.bash

#Run the node
ros2 run mocap_optitrack_w2b mocap_optitrack_w2b
