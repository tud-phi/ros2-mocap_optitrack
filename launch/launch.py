from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    #Create the NatNet client node
    config = os.path.join(
        get_package_share_directory('mocap_optitrack_client'),
        'config',
        'natnetclient.yaml'
        )
    natnet_client = Node(
            package='mocap_optitrack_client',
            executable='mocap_optitrack_client',
            name='natnet_client',
            parameters = [config]
        )
    #Create the world to base client
    config = os.path.join(
        get_package_share_directory('mocap_optitrack_w2b'),
        'config',
        'world_to_base.yaml'
        )
    world_to_base = Node(
            package='mocap_optitrack_w2b',
            executable='mocap_optitrack_w2b',
            name='world_to_base',
            parameters = [config]
        )
    #Add the nodes to the launch description and return it
    ld.add_action(natnet_client)
    ld.add_action(world_to_base)
    return ld