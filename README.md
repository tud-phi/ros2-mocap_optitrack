# ROS2 MoCap Optitrack
Set of ROS2 packages implementing the data acquisition system described in: https://www.overleaf.com/read/mggtpwfjhjzt

## Configuration of NatNet server
Please first read the [Streaming Guide](https://v22.wiki.optitrack.com/index.php?title=Data_Streaming) by Optitrack to get yourself familiar with the topic.

Make sure that a static IP is used on the Optitrack (Windows) workstation for the LAN connection with the Optitrack switch. **Windows Defender needs to be deactivated for all networks as well!**. Nominally, the static IP should be set to `192.168.4.31`, which is the same local network as of the VTEM terminal. 

In the streaming pane of Motive, switch-on _Broadcast Frame Data_ and select as the local interface `192.168.4.31`. As transmission type select _Multicast_.

On your target machine (usually the Ubuntu lab workstation), set a static IP to the local network as well (for example `192.168.4.20`) and try to pin the source workstation with `ping 192.168.4.31`. The IP address of the source also needs to be set in the `mocap_optitrack_client/config/natnetclient.yaml` file.


## Usage
Build the packages: 
```bash
colcon build --packages-select mocap_optitrack_client mocap_optitrack_inv_kin mocap_optitrack_w2b
```

Source the built packages:
```bash
. install/setup.bash
```

Move to the source directory and run the system via the configuration file:
```bash
ros2 launch launch/launch_y_up.py
```
