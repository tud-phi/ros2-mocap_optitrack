from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    #config = os.path.join(
    #    get_package_share_directory('ros2_tutorials'),
    #    'config',
    #    'params.yaml'
    #    )
    #Create the NatNet client node
    natnet_client = Node(
            package='mocap_optitrack_client',
            executable='mocap_optitrack_client',
            name='natnet_client'
        )
    #Create the world to base client
    world_to_base = Node(
            package='mocap_optitrack_w2b',
            executable='mocap_optitrack_w2b',
            name='world_to_base'
        )
    #Add the nodes to the launch description and return it
    ld.add_action(natnet_client)
    ld.add_action(world_to_base)
    return ld