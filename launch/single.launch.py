import os
from launch import LaunchDescription
import yaml
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

params = os.path.join(
    get_package_share_directory('farmbot_localization'),
    'config',
    'params.yaml'
)

global_params = yaml.safe_load(open(params))['global']['ros__parameters']

def generate_launch_description():
    ld = LaunchDescription()

    antenna_split = Node(
        package='farmbot_localization',
        executable='antenna_split',
        name='antenna_split',
        parameters=[params, global_params]
    )

    antenna_fuse = Node(
        package='farmbot_localization',
        executable='antenna_fuse',
        name='antenna_fuse',
        parameters=[params, global_params]
    )

    gps_to_enu = Node(
        package='farmbot_localization',
        executable='gps_to_enu',
        name='gps_to_enu',
        parameters=[params, global_params]
    )

    odom_n_path = Node(
        package='farmbot_localization',
        executable='odom_n_path',
        name='odom_n_path',
        parameters=[params, global_params]
    )

    transform_pub = Node(
        package='farmbot_localization',
        executable='transform_pub',
        name='transform_pub',
        parameters=[params, global_params]
    )

    # ld.add_action(antenna_split)
    # ld.add_action(antenna_fuse)
    # ld.add_action(gps_to_enu)
    ld.add_action(odom_n_path)
    # ld.add_action(transform_pub)

    return ld