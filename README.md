Set of ROS2 packages implementing the data acquisition system described in: https://www.overleaf.com/read/mggtpwfjhjzt

Build the packages: 
```bash
colcon build --packages-select mocap_optitrack_client mocap_optitrack_inv_kin mocap_optitrack_w2b
```

Source the built packages:
```bash
. install/setup.bash
```
Enter the source directory:

Run the system via the configuratin file:
```bash
ros2 launch launch/launch.py
```
