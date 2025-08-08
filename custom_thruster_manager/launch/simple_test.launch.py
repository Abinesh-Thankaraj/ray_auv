#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('custom_thruster_manager')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='ray',
        description='Namespace for the Ray'
    )
    
    # Custom thruster manager node
    custom_thruster_manager_node = Node(
        package='custom_thruster_manager',
        executable='custom_thruster_manager_node',
        name='custom_thruster_manager',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            os.path.join(pkg_dir, 'config', 'custom_thruster_manager.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_namespace,
        custom_thruster_manager_node
    ])
