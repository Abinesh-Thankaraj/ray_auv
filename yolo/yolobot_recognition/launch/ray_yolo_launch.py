#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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
    
    return LaunchDescription([
        use_sim_time_arg,
        confidence_threshold_arg,
        device_arg,
        yolo_detector_node,
    ])
