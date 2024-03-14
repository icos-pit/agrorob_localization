import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import yaml
import argparse
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    param_file = os.path.join(get_package_share_directory('farmbot_localization'), 'config', 'params.yaml')
    
    antenna_split = Node(
        package='farmbot_localization',
        condition = UnlessCondition(LaunchConfiguration('double_antenna')),
        namespace=namespace,
        executable='antenna_split',
        name='antenna_split',
        parameters=[
            {"frame_prefix": namespace+"/"},
            {"namespace": namespace},
            yaml.safe_load(open(param_file))['antenna_split']['ros__parameters'], 
            yaml.safe_load(open(param_file))['global']['ros__parameters']
        ]
    )

    antenna_fuse = Node(
        package='farmbot_localization',
        namespace=namespace,
        executable='antenna_fuse',
        name='antenna_fuse',
        parameters=[
            {"frame_prefix": namespace+"/"},
            {"namespace": namespace},
            yaml.safe_load(open(param_file))['antenna_fuse']['ros__parameters'], 
            yaml.safe_load(open(param_file))['global']['ros__parameters']
        ]    
    )

    gps_to_enu = Node(
        package='farmbot_localization',
        namespace=namespace,
        executable='gps_to_enu',
        name='gps_to_enu',
        parameters=[
            {"frame_prefix": namespace+"/"},
            {"namespace": namespace},
            yaml.safe_load(open(param_file))['gps_to_enu']['ros__parameters'], 
            yaml.safe_load(open(param_file))['global']['ros__parameters'],
        ]    
    )

    odom_n_path = Node(
        package='farmbot_localization',
        namespace=namespace,
        executable='odom_n_path',
        name='odom_n_path',
        parameters=[
            {"frame_prefix": namespace+"/"},
            {"namespace": namespace},
            yaml.safe_load(open(param_file))['odom_n_path']['ros__parameters'], 
            yaml.safe_load(open(param_file))['global']['ros__parameters']
        ]    
    )

    transform_pub = Node(
        package='farmbot_localization',
        namespace=namespace,
        executable='transform_pub',
        name='transform_pub',
        parameters=[
            {"frame_prefix": namespace+"/"},
            {"namespace": namespace},
            yaml.safe_load(open(param_file))['transform_pub']['ros__parameters'], 
            yaml.safe_load(open(param_file))['global']['ros__parameters']
        ]
    )
    return [ 
        antenna_split,
        antenna_fuse,
        gps_to_enu,
        odom_n_path,
        transform_pub,
    ]


def generate_launch_description(): 
    namespace_arg = DeclareLaunchArgument('namespace', default_value='fb')
    antena_arg = DeclareLaunchArgument('double_antenna', default_value='True')
    
    return LaunchDescription([
        namespace_arg,
        antena_arg, 
        OpaqueFunction(function = launch_setup)
    ])