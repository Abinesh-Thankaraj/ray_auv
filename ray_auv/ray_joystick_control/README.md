# RAY AUV PS5 Joystick Control System

Complete PS5 DualSense joystick controller integration for RAY AUV 6-DOF motion control.

## Features

- **6-DOF Motion Control**: Full control over all 6 degrees of freedom (X, Y, Z, Roll, Pitch, Yaw)
- **Dual-Stick Control**: Left stick for lateral/forward movement, right stick for rotation
- **Trigger-Based Depth Control**: L2/R2 triggers for ascending/descending
- **Mode Toggle**: X button to switch between roll/pitch and yaw control modes
- **Deadzone Filtering**: Configurable deadzone for analog sticks to prevent drift
- **Real-time Publishing**: 20 Hz update rate for smooth control

## Installation & Setup

### 1. Prerequisites

Ensure you have the PS5 DualSense joystick connected to your system and the ROS 2 `joy` package installed:

```bash
sudo apt-get update
sudo apt-get install ros-<distro>-joy
```

Replace `<distro>` with your ROS 2 distribution (e.g., `humble`, `iron`).

### 2. Build the Package

```bash
cd /home/pdc/ray_ws
colcon build --packages-select ray_joystick_control
source install/setup.bash
```

### 3. Verify Joystick Connection

Check if your joystick is recognized:

```bash
ls -l /dev/input/js*
```

You should see one or more joystick devices (typically `/dev/input/js0`).

### 4. Test Joystick Input (Optional)

Install and run the `jstest` utility to verify joystick functionality:

```bash
sudo apt-get install joystick
jstest /dev/input/js0
```

## Usage

### Launch with Joystick Control

```bash
# Full simulation with visualization (Gazebo + RViz)
ros2 launch ray_description canyon_world_launch.py

# In another terminal, launch joystick control
ros2 launch ray_joystick_control joystick_launch.py
```

### Alternative: Without RViz

```bash
ros2 launch ray_joystick_control joystick_launch.py rviz:=false
```

### With Custom Joystick Device

If your joystick is on a different device (e.g., `/dev/input/js1`):

```bash
ros2 launch ray_joystick_control joystick_launch.py joy_dev:=/dev/input/js1
```

## Joystick Control Mapping

### PS5 DualSense Controller Layout

```
                    △
                   /|\
                  / | \
                 /  |  \
          □ ----+---+---+---- ○
         /       \  |  /       \
        /         \ | /         \
    (L1) --------- \|/ --------- (R1)
(L2) L2 TRIGGER        R2 TRIGGER (R2)
    (L3)   Left Stick | Right Stick   (R3)
           ↖ ↑ ↗      |      ↖ ↑ ↗
           ← X →      |      ← X →
           ↙ ↓ ↘      |      ↙ ↓ ↘
                      X (Mode Toggle)
```

### Axis Mappings

| Input | Function | Motion DOF |
|-------|----------|-----------|
| **Left Stick X** | Lateral movement (left/right) | Y-axis |
| **Left Stick Y** | Forward/backward movement | X-axis |
| **L2 Trigger** | Descend (increase depth) | Z-axis (negative) |
| **R2 Trigger** | Ascend (decrease depth) | Z-axis (positive) |
| **Right Stick X** | Roll (normal) / Yaw (mode toggle) | Roll or Yaw |
| **Right Stick Y** | Pitch rotation | Pitch |
| **X Button** | Toggle between Roll/Pitch ↔ Yaw mode | Mode |

### Motion Control Modes

#### Normal Mode (Roll/Pitch Control)
- **Right Stick X**: Controls Roll rotation
- **Right Stick Y**: Controls Pitch rotation
- Ideal for fine-tuning orientation

#### Yaw Mode (Yaw Control)
- **Press X Button** to activate yaw mode
- **Right Stick X**: Controls Yaw rotation
- **Right Stick Y**: Still controls Pitch
- Press **X Button again** to return to normal mode

### Complete Movement Examples

**Move Forward & Descend:**
- Push Left Stick Y forward
- Push L2 trigger halfway

**Rotate Right While Moving Left:**
- Push Left Stick X left
- In normal mode: Push Right Stick X right for roll
- In yaw mode: Push Right Stick X right for yaw

**Stabilize Depth While Rotating:**
- Adjust L2/R2 to maintain depth
- Use Right Stick for pitch/roll control

## Configuration

Edit `/home/pdc/ray_ws/src/ray_auv/ray_joystick_control/config/joystick_controller.yaml` to customize:

```yaml
joystick_controller:
  ros__parameters:
    deadzone: 0.1                    # Deadzone threshold (0.0-1.0)
    max_linear_velocity: 1.0          # Max m/s for X, Y, Z motion
    max_angular_velocity: 1.5708      # Max rad/s for rotations (~90°/s)
    max_depth: 20.0                   # Maximum depth in meters
    initial_z: -5.0                   # Starting depth in meters
```

### Parameter Descriptions

- **deadzone**: Prevents joystick drift by ignoring inputs below this threshold
- **max_linear_velocity**: Speed limit for forward/lateral/vertical motion
- **max_angular_velocity**: Speed limit for roll/pitch/yaw rotation
- **max_depth**: Hard limit for how deep the ROV can go (negative Z)
- **initial_z**: Starting Z position when the node launches

## Troubleshooting

### Joystick Not Detected

```bash
# Check permissions
ls -l /dev/input/js0

# Add user to input group if needed
sudo usermod -a -G input $USER
newgrp input
```

### Inverted Controls

Some joysticks may have inverted axes. Edit the `joystick_controller.py` script:
- Change line with `left_stick_y = -self.apply_deadzone(msg.axes[1])` to remove the `-` sign
- Similarly for right stick Y axis if needed

### ROV Not Responding

1. Verify the joystick node is publishing:
   ```bash
   ros2 topic echo /joy
   ```

2. Check if joystick_controller is receiving messages:
   ```bash
   ros2 topic echo /ray/cmd_pose
   ```

3. Verify the body_control_sm node is active:
   ```bash
   ros2 node list
   ros2 node info /ray/body_control_sm
   ```

### Joystick Axes Different Than Expected

Run `jstest /dev/input/js0` to identify your controller's specific axis layout and map accordingly.

## Topic Reference

### Published Topics

- **`/ray/cmd_pose`** (geometry_msgs/PoseStamped)
  - Contains target position (X, Y, Z) and orientation (roll, pitch, yaw as quaternion)
  - Published at 20 Hz
  - Consumed by the body_control_sm (auv_control) node

### Subscribed Topics

- **`/joy`** (sensor_msgs/Joy)
  - Raw joystick input from the joy_node
  - Button and axis data from PS5 controller

## System Architecture

```
PS5 DualSense Controller
         |
         ↓
   /dev/input/js0
         |
         ↓
   joy_node (ROS 2 Joy Package)
   publishes: /joy
         |
         ↓
   joystick_controller.py (This Node)
   subscribes: /joy
   publishes: /ray/cmd_pose
         |
         ↓
   body_control_sm (auv_control)
   subscribes: /ray/cmd_pose
   calculates control forces
         |
         ↓
   ray_thruster_manager
   allocates forces to 6 thrusters
         |
         ↓
   Gazebo Thrusters (Simulation)
   or Real Thrusters (Hardware)
```

## Advanced Usage

### Launch Without Simulation

If running on real hardware without Gazebo:

```bash
# Modify joystick_launch.py to remove use_sim_time or set it to false
# Then launch:
ros2 launch ray_joystick_control joystick_launch.py use_sim_time:=false
```

### Custom Joystick Device on Startup

Create a launch argument and modify the launch file:

```bash
ros2 launch ray_joystick_control joystick_launch.py joy_dev:=/dev/input/js1
```

## Performance Tuning

### For Slower Systems
- Increase deadzone to reduce noise processing
- Decrease publishing frequency in the joy node config
- Reduce max_angular_velocity for smoother control

### For More Responsive Control
- Decrease deadzone (but watch for drift)
- Increase max_linear_velocity and max_angular_velocity
- Ensure no other heavy processes are consuming CPU

## File Structure

```
ray_joystick_control/
├── CMakeLists.txt                 # Build configuration
├── package.xml                    # Package metadata
├── scripts/
│   └── joystick_controller.py    # Main controller node
├── config/
│   └── joystick_controller.yaml  # Configuration parameters
├── launch/
│   └── joystick_launch.py        # Launch file
└── README.md                      # This file
```

## Future Enhancements

- [ ] Support for multiple joystick profiles (Xbox, Switch Pro, etc.)
- [ ] Dead man switch mode for safety
- [ ] Velocity vs. position control toggle
- [ ] Camera/gimbal control mapping
- [ ] Recording and playback of joystick inputs
- [ ] Force feedback support for haptic feedback

## Support & Debugging

For issues or questions:

1. Check the ROS 2 logs:
   ```bash
   ros2 launch ray_joystick_control joystick_launch.py
   ```

2. Enable debug logging:
   ```bash
   export ROS_LOG_LEVEL=DEBUG
   ros2 launch ray_joystick_control joystick_launch.py
   ```

3. Verify message flow:
   ```bash
   ros2 topic list
   ros2 topic echo /joy
   ros2 topic echo /ray/cmd_pose
   ```

## License

MIT - Same as RAY AUV project

## Author

Developed for RAY AUV autonomous underwater vehicle control system.
