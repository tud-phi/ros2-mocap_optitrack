#Launch file when in the data streaming pane the Up Axis option is set to "Y up"
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    log_level = "warn"

    ld = LaunchDescription()
    # Create the NatNet client node
    config = os.path.join(
        get_package_share_directory('mocap_optitrack_client'),
        'config',
        'natnetclient.yaml'
    )
    natnet_client = Node(
        package='mocap_optitrack_client',
        executable='mocap_optitrack_client',
        name='natnet_client',
        parameters = [config],
        arguments=['--ros-args', '--log-level', log_level]
    )
    # Create the world to base client
    config = os.path.join(
        get_package_share_directory('mocap_optitrack_w2b'),
        'config',
        'world_to_base_y_up.yaml'
    )
    world_to_base = Node(
        package='mocap_optitrack_w2b',
        executable='mocap_optitrack_w2b',
        name='world_to_base',
        parameters = [config],
        arguments=['--ros-args', '--log-level', log_level]
    )
    # Create the inverse kinematics node
    config = os.path.join(
        get_package_share_directory('mocap_optitrack_inv_kin'),
        'config',
        'inverse_kinematics.yaml'
    )
    inverse_kinematics = Node(
        package='mocap_optitrack_inv_kin',
        executable='mocap_optitrack_inv_kin',
        name='inverse_kinematics',
        parameters = [config],
        arguments=['--ros-args', '--log-level', log_level]
    )
    # Add the nodes to the launch description and return it
    ld.add_action(natnet_client)
    ld.add_action(world_to_base)
    ld.add_action(inverse_kinematics)
    return ld