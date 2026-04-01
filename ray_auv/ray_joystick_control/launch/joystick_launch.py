from simple_launch import SimpleLauncher


def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time=True)
    
    sl.declare_arg('namespace', default_value='ray')
    sl.declare_arg('use_joystick', default_value=True)
    sl.declare_arg('rviz', default_value=True)
    sl.declare_arg('joy_dev', default_value='/dev/input/js0')
    
    with sl.group(ns=sl.arg('namespace')):
        
        # Launch ray thruster manager
        sl.node('ray_thruster_manager', 'ray_thruster_manager_node',
                name='ray_thruster_manager',
                parameters=[sl.find('ray_thruster_manager', 'ray_thruster_manager.yaml')],
                output='screen')
        
        # Load body controller with custom configuration
        sl.node('auv_control', 'sliding_mode',
                parameters=[sl.find('ray_control', 'custom_sliding_mode.yaml')],
                output='screen')
        
        # Launch pressure sensor for depth and bar pressure readings
        sl.node('ray_description', 'ray_pressure_sensor.py',
                name='ray_pressure_sensor',
                output='log')
        
        # Launch joystick node from joy package
        sl.node('joy', 'joy_node',
                name='joy',
                parameters=[{'dev': sl.arg('joy_dev')}],
                output='screen')
        
        # Launch joystick controller node
        sl.node('ray_joystick_control', 'joystick_controller.py',
                name='joystick_controller',
                parameters=[sl.find('ray_joystick_control', 'joystick_controller.yaml')],
                output='screen')
    
    # Optional RViz visualization
    with sl.group(if_arg='rviz'):
        sl.include('ray_control', 'rviz_launch.py',
                   launch_arguments={'namespace': sl.arg('namespace'), 'use_sim_time': sl.sim_time})
    
    return sl.launch_description()
