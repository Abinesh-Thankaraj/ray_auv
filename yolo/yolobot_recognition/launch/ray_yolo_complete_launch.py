#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import os

def generate_launch_description():
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Minimum confidence threshold for person detection'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cpu',  # Use CPU by default, change to 'cuda:0' if GPU available
        description='Device to use for inference (cpu or cuda:0)'
    )
    
    show_display_arg = DeclareLaunchArgument(
        'show_display',
        default_value='true',
        description='Show visual display window (true/false)'
    )
    
    # YOLO detector node
    yolo_detector_node = Node(
        package='yolobot_recognition',
        executable='yolov8_ray_detector.py',
        name='ray_yolo_detector',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        arguments=[
            '--confidence_threshold', LaunchConfiguration('confidence_threshold'),
            '--device', LaunchConfiguration('device'),
        ]
    )
    
    # YOLO monitor node (conditional)
    yolo_monitor_node = Node(
        package='yolobot_recognition',
        executable='yolo_monitor.py',
        name='yolo_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'show_display': LaunchConfiguration('show_display'),
        }],
        condition=IfCondition(LaunchConfiguration('show_display'))
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        confidence_threshold_arg,
        device_arg,
        show_display_arg,
        yolo_detector_node,
        yolo_monitor_node,
    ])
