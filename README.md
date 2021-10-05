ROS2 packages implementaing the data acquisition system described in: //TO DO : add link to paper

Build the packages: 
```bash
colcon build --packages-select mocap_optitrack_client mocap_optitrack_inv_kin mocap_optitrack_w2b
```

Source the built packages:
```bash
. install/setup.bash
```

//TO DO: remove the NatNet SDK dependency
Save the NatNet SDK library to the path:
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/pietro/Scrivania/TESI/ROS2/optitrack/ros2-mocap_optitrack/mocap_optitrack_client/lib
```

Run the system via the configuratin file:
```bash
ros2 launch launch/launch.py
```
