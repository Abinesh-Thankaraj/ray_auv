#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Confidence threshold for YOLO detections'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # YOLO Detector with Display Node
    yolo_detector_display_node = Node(
        package='yolobot_recognition',
        executable='yolov8_ray_detector_with_display.py',
        name='ray_yolo_detector_display',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold')
        }]
    )

    return LaunchDescription([
        confidence_threshold_arg,
        use_sim_time_arg,
        yolo_detector_display_node
    ])
