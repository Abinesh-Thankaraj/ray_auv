#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
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
    
    declare_sliders = DeclareLaunchArgument(
        'sliders',
        default_value='true',
        description='Launch slider publishers'
    )
    
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
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
    
    # Sliding mode controller
    sliding_mode_node = Node(
        package='auv_control',
        executable='sliding_mode',
        name='body_control_sm',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            os.path.join(pkg_dir, 'config', 'custom_sliding_mode.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )
    
    # Pose control slider
    pose_control_node = Node(
        package='slider_publisher',
        executable='slider_publisher',
        name='pose_control',
        namespace=LaunchConfiguration('namespace'),
        arguments=[os.path.join(get_package_share_directory('auv_control'), 'launch', 'pose_setpoint.yaml')],
        condition=IfCondition(LaunchConfiguration('sliders')),
        output='screen'
    )
    
    # Tilt control slider
    tilt_control_node = Node(
        package='slider_publisher',
        executable='slider_publisher',
        name='tilt_control',
        namespace=LaunchConfiguration('namespace'),
        arguments=[os.path.join(get_package_share_directory('ray_control'), 'config', 'tilt.yaml')],
        condition=IfCondition(LaunchConfiguration('sliders')),
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_namespace,
        declare_sliders,
        declare_rviz,
        custom_thruster_manager_node,
        sliding_mode_node,
        pose_control_node,
        tilt_control_node
    ])
