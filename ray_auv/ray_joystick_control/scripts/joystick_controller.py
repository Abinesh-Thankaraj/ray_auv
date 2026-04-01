#!/usr/bin/env python3
"""
RAY AUV PS5 Joystick Controller Node
Converts PS5 DualSense controller input to ROV 6-DOF motion commands.

Joystick Mapping:
- Left Stick X/Y: Lateral (X) and Forward/Backward (Y) motion
- Right Stick X/Y: Roll and Pitch rotation
- L2/R2 Triggers: Depth control (down/up)
- X Button: Activate yaw control mode, then Right Stick X: Yaw rotation
- Deadzone handling for all axes
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import math

class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_controller')
        
        # Declare parameters - ULTRA RESPONSIVE MODE for fast ROV simulation
        self.declare_parameter('deadzone', 0.02)
        self.declare_parameter('max_linear_velocity', 5.0)
        self.declare_parameter('max_angular_velocity', 6.28318)  # 2*pi rad/s (fast rotation)
        self.declare_parameter('max_depth', 20.0)  # meters (negative for going down)
        self.declare_parameter('initial_z', 0.0)  # initial depth in meters (0 = water surface)
        self.declare_parameter('smoothing_factor', 0.95)  # 0-1: higher = more responsive
        
        # Get parameters
        self.deadzone = self.get_parameter('deadzone').value
        self.max_lin_vel = self.get_parameter('max_linear_velocity').value
        self.max_ang_vel = self.get_parameter('max_angular_velocity').value
        self.max_depth = self.get_parameter('max_depth').value
        self.initial_z = self.get_parameter('initial_z').value
        self.smoothing = self.get_parameter('smoothing_factor').value
        
        # Current state
        self.current_pose = {
            'x': 0.0,
            'y': 0.0,
            'z': self.initial_z,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0
        }
        
        # Current velocity (for smooth game-like motion)
        self.current_velocity = {
            'x': 0.0,  # lateral velocity
            'y': 0.0,  # forward velocity
            'z': 0.0,  # depth velocity
            'roll': 0.0,  # roll rate
            'pitch': 0.0,  # pitch rate
            'yaw': 0.0   # yaw rate
        }
        
        # Yaw control mode flag
        self.yaw_mode = False
        self.x_button_pressed = False
        
        # Create subscription to joy topic
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
        
        # Create publisher for pose commands
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pose_pub = self.create_publisher(
            PoseStamped, 'cmd_pose', qos_profile)
        
        # Create timer for publishing pose at regular intervals (50 Hz for smooth control)
        self.timer = self.create_timer(0.02, self.publish_pose)  # 50 Hz
        self.last_update_time = self.get_clock().now()
        
        self.get_logger().info('🎮 Joystick Controller Initialized (SMOOTH MODE)')
        self.get_logger().info(f'📊 Deadzone: {self.deadzone} (Low = More Responsive)')
        self.get_logger().info(f'⚡ Max Linear Velocity: {self.max_lin_vel} m/s')
        self.get_logger().info(f'⚡ Max Angular Velocity: {self.max_ang_vel:.2f} rad/s')
        self.get_logger().info(f'📍 Starting Position: z = {self.initial_z}m')
        self.get_logger().info(f'🎯 Smoothing Factor: {self.smoothing} (Higher = More Responsive)')
        self.get_logger().info('👂 Waiting for joystick input...')
        
    def apply_deadzone(self, value):
        """Apply deadzone to analog stick input"""
        if abs(value) < self.deadzone:
            return 0.0
        # Scale the value to account for deadzone
        if value > 0:
            return (value - self.deadzone) / (1.0 - self.deadzone)
        else:
            return (value + self.deadzone) / (1.0 - self.deadzone)
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return (qx, qy, qz, qw)
    
    def joy_callback(self, msg: Joy):
        """
        PS5 DualSense Button/Axis Mapping:
        Axes:
          0: Left Stick X (lateral motion, -1=left, 1=right)
          1: Left Stick Y (forward/backward, -1=forward, 1=backward)
          2: L2 Trigger (-1=not pressed, 1=fully pressed)
          3: Right Stick X (roll/yaw depending on mode, -1=left, 1=right)
          4: Right Stick Y (pitch, -1=up, 1=down)
          5: R2 Trigger (-1=not pressed, 1=fully pressed)
        
        Buttons:
          0: Square
          1: X (Triangle in PlayStation numbering)
          2: Circle
          3: Triangle
          4: LB (L1)
          5: RB (R1)
          6: LT (L2 button)
          7: RT (R2 button)
          8: Left Stick Click
          9: Right Stick Click
          10: PS Button
          11: Touchpad Click
          12: MIC Button
        """
        try:
            # Apply deadzone to joystick inputs
            left_stick_x = self.apply_deadzone(msg.axes[0])  # Forward/backward motion (SWAPPED)
            left_stick_y = -self.apply_deadzone(msg.axes[1])  # Lateral motion (SWAPPED - inverted)
            right_stick_x = self.apply_deadzone(msg.axes[3])  # Yaw or Roll
            right_stick_y = -self.apply_deadzone(msg.axes[4])  # Pitch (inverted)
            l2_trigger = self.apply_deadzone((msg.axes[2] + 1) / 2)  # Convert to 0-1 range
            r2_trigger = self.apply_deadzone((msg.axes[5] + 1) / 2)  # Convert to 0-1 range
            
            # Check X button (button 1) for yaw mode toggle - DEPRECATED, using L1/R1 now
            # if msg.buttons[1] == 1 and not self.x_button_pressed:
            #     self.yaw_mode = not self.yaw_mode
            #     self.x_button_pressed = True
            #     mode_str = "🎯 YAW MODE" if self.yaw_mode else "⚙️  ROLL/PITCH MODE"
            #     self.get_logger().info(f'Switched to {mode_str}')
            # elif msg.buttons[1] == 0:
            #     self.x_button_pressed = False
            
            # SMOOTH VELOCITY-BASED CONTROL (Game-like responsiveness)
            # Update target velocities directly from joystick input
            
            # Left stick: forward/backward (Y-axis vertical) = Surge, left/right (X-axis horizontal) = Sway (REVERSED)
            target_vel_x = left_stick_y * self.max_lin_vel  # Forward/backward stick → Surge (X motion)
            target_vel_y = -left_stick_x * self.max_lin_vel  # Left/right stick (REVERSED) → Sway (Y motion)
            
            # Smoothly interpolate velocity to target (exponential smoothing)
            self.current_velocity['x'] = self.current_velocity['x'] * (1 - self.smoothing) + target_vel_x * self.smoothing
            self.current_velocity['y'] = self.current_velocity['y'] * (1 - self.smoothing) + target_vel_y * self.smoothing
            
            # Triggers: depth control (Z axis) - REVERSED
            # L2 increases depth (going up, positive Z)
            # R2 decreases depth (going down, negative Z)
            target_vel_z = (l2_trigger - r2_trigger) * self.max_lin_vel
            self.current_velocity['z'] = self.current_velocity['z'] * (1 - self.smoothing) + target_vel_z * self.smoothing
            
            # L1/R1 Buttons: Yaw control (Rotation Left/Right) - REVERSED
            # L1 (button 4): Yaw right (positive rotation)
            # R1 (button 5): Yaw left (negative rotation)
            l1_pressed = msg.buttons[4] if len(msg.buttons) > 4 else 0
            r1_pressed = msg.buttons[5] if len(msg.buttons) > 5 else 0
            
            if l1_pressed:
                target_vel_yaw = self.max_ang_vel  # Turn right
            elif r1_pressed:
                target_vel_yaw = -self.max_ang_vel  # Turn left
            else:
                target_vel_yaw = 0.0  # No rotation
            
            # Smoothly interpolate yaw velocity
            self.current_velocity['yaw'] = self.current_velocity['yaw'] * (1 - self.smoothing) + target_vel_yaw * self.smoothing
            
            # Right stick: Roll and Pitch control (always active)
            target_vel_roll = right_stick_x * self.max_ang_vel
            target_vel_pitch = right_stick_y * self.max_ang_vel
            
            self.current_velocity['roll'] = self.current_velocity['roll'] * (1 - self.smoothing) + target_vel_roll * self.smoothing
            self.current_velocity['pitch'] = self.current_velocity['pitch'] * (1 - self.smoothing) + target_vel_pitch * self.smoothing
                    
        except (IndexError, AttributeError) as e:
            self.get_logger().warn(f'Joy message format issue: {e}')
    
    def publish_pose(self):
        """Publish current pose as PoseStamped message"""
        
        # Calculate dt for integration
        current_time = self.get_clock().now()
        if not hasattr(self, 'last_publish_time'):
            self.last_publish_time = current_time
            dt = 0.02
        else:
            dt = (current_time - self.last_publish_time).nanoseconds / 1e9
            self.last_publish_time = current_time
        
        # Clamp dt to reasonable range to prevent jumps
        dt = max(0.005, min(0.1, dt))
        
        # Integrate velocity to position (Euler integration)
        self.current_pose['x'] += self.current_velocity['x'] * dt
        self.current_pose['y'] += self.current_velocity['y'] * dt
        self.current_pose['z'] += self.current_velocity['z'] * dt
        
        # Clamp depth to max range [−max_depth, 0]
        self.current_pose['z'] = max(-self.max_depth, min(0.0, self.current_pose['z']))
        
        # Integrate angular velocity to angles
        self.current_pose['roll'] += self.current_velocity['roll'] * dt
        self.current_pose['pitch'] += self.current_velocity['pitch'] * dt
        self.current_pose['yaw'] += self.current_velocity['yaw'] * dt
        
        # Clamp angles to [-pi, pi]
        for angle in ['roll', 'pitch', 'yaw']:
            while self.current_pose[angle] > math.pi:
                self.current_pose[angle] -= 2 * math.pi
            while self.current_pose[angle] < -math.pi:
                self.current_pose[angle] += 2 * math.pi
        
        # Create PoseStamped message
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        
        # Position
        msg.pose.position.x = self.current_pose['x']
        msg.pose.position.y = self.current_pose['y']
        msg.pose.position.z = self.current_pose['z']
        
        # Orientation as quaternion
        qx, qy, qz, qw = self.euler_to_quaternion(
            self.current_pose['roll'],
            self.current_pose['pitch'],
            self.current_pose['yaw']
        )
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        
        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    joystick_controller = JoystickController()
    
    try:
        rclpy.spin(joystick_controller)
    except KeyboardInterrupt:
        pass
    finally:
        joystick_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
