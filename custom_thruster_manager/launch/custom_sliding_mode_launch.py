from simple_launch import SimpleLauncher


def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time = True)
    
    sl.declare_arg('namespace', default_value='ray')
    sl.declare_arg('sliders', default_value=True)
    sl.declare_arg('rviz', default_value=True)
    
    with sl.group(ns=sl.arg('namespace')):
        
        # Load custom thruster manager instead of default
        sl.node('custom_thruster_manager', 'custom_thruster_manager_node', 
                parameters=[sl.find('custom_thruster_manager', 'custom_thruster_manager.yaml')],
                output='screen')
        
        # Load body controller with custom thruster manager
        sl.node('auv_control', 'sliding_mode', parameters=[sl.find('custom_thruster_manager', 'custom_sliding_mode.yaml')],
                output='screen')

        with sl.group(if_arg='sliders'):
            sl.node('slider_publisher', 'slider_publisher', name='pose_control',
                    arguments=[sl.find('auv_control', 'pose_setpoint.yaml')])

            sl.node('slider_publisher', 'slider_publisher', name='tilt_control',
                    arguments=[sl.find('ray_control', 'tilt.yaml')])

    with sl.group(if_arg='rviz'):
        sl.include('ray_control','rviz_launch.py',
                   launch_arguments={'namespace': sl.arg('namespace'), 'use_sim_time': sl.sim_time})

    return sl.launch_description()
