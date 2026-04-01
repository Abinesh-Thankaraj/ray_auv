from simple_launch import SimpleLauncher


def generate_launch_description():
    """
    Simplified launch file for joystick control only
    Assumes the simulation (canyon_world_launch.py) is already running
    and body controller is launched separately with custom_sliding_mode_launch.py
    
    This file only launches the joystick input node.
    """
    
    sl = SimpleLauncher(use_sim_time=True)
    
    sl.declare_arg('namespace', default_value='ray')
    sl.declare_arg('joy_dev', default_value='/dev/input/js0')
    sl.declare_arg('joy_frequency', default_value='30')
    
    with sl.group(ns=sl.arg('namespace')):
        
        # Launch joystick node from joy package
        sl.node('joy', 'joy_node',
                name='joy',
                parameters=[{
                    'dev': sl.arg('joy_dev'),
                    'frequency': float(sl.arg('joy_frequency'))
                }],
                output='screen')
        
        # Launch joystick controller node
        sl.node('ray_joystick_control', 'joystick_controller.py',
                name='joystick_controller',
                parameters=[sl.find('ray_joystick_control', 'joystick_controller.yaml')],
                output='screen')
    
    return sl.launch_description()
