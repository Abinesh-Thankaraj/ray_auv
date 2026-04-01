# Quick Start Guide: PS5 Joystick Control for RAY AUV

## 5-Minute Setup

### Step 1: Connect PS5 Controller
1. Press the PS button + Share button simultaneously to enter pairing mode
2. On your system, go to Bluetooth settings and pair the DualSense controller
3. Verify it's connected:
   ```bash
   ls /dev/input/js*
   ```
   Should show `/dev/input/js0` or similar

### Step 2: Build the Package
```bash
cd /home/pdc/ray_ws
colcon build --packages-select ray_joystick_control
source install/setup.bash
```

### Step 3: Run the Simulation
Terminal 1:
```bash
ros2 launch ray_description canyon_world_launch.py
```

Terminal 2:
```bash
ros2 launch ray_control custom_sliding_mode_launch.py
```

Terminal 3:
```bash
ros2 launch ray_joystick_control joystick_launch.py
```

## Control Your ROV

| Action | Joystick Input |
|--------|-----------------|
| **Move Forward** | Push Left Stick UP |
| **Move Backward** | Push Left Stick DOWN |
| **Strafe Left** | Push Left Stick LEFT |
| **Strafe Right** | Push Left Stick RIGHT |
| **Go Deeper** | Press L2 Trigger |
| **Go Shallower** | Press R2 Trigger |
| **Pitch Down** | Push Right Stick DOWN |
| **Pitch Up** | Push Right Stick UP |
| **Roll Left** | Push Right Stick LEFT |
| **Roll Right** | Push Right Stick RIGHT |
| **Rotate Left (Yaw)** | Press X, then Left Stick RIGHT |
| **Rotate Right (Yaw)** | Press X, then Right Stick LEFT |
| **Exit Yaw Mode** | Press X again |

## All-In-One Launch (Complete System)

If you want a single launch file that starts everything:

```bash
ros2 launch ray_joystick_control joystick_launch.py
```

This includes:
- ✅ Gazebo simulation (canyon world)
- ✅ Body controller
- ✅ Thruster manager
- ✅ Pressure sensor
- ✅ Joystick input
- ✅ RViz visualization

## Troubleshooting

**Problem:** "Cannot open /dev/input/js0"
- Run `sudo chmod a+rw /dev/input/js0`

**Problem:** Joystick not responding
- Check: `ros2 topic echo /joy` - should show controller input
- Check: `ros2 topic echo /ray/cmd_pose` - should show position updates

**Problem:** Controls feel inverted
- Edit [joystick_controller.yaml](config/joystick_controller.yaml)
- Adjust `deadzone` and `max_linear_velocity`

## Advanced: Separate Launches

If you prefer running components separately:

```bash
# Terminal 1: Start simulation
ros2 launch ray_description canyon_world_launch.py

# Terminal 2: Start body control
ros2 launch ray_control custom_sliding_mode_launch.py

# Terminal 3: Start ONLY joystick input (no redundant nodes)
ros2 launch ray_joystick_control joystick_only_launch.py
```

## Configuration Tips

Edit `/home/pdc/ray_ws/src/ray_auv/ray_joystick_control/config/joystick_controller.yaml`:

```yaml
# Make it faster
max_linear_velocity: 2.0        # Increased from 1.0
max_angular_velocity: 3.14      # Increased from 1.57 (~180°/s)

# Make it more responsive
deadzone: 0.05                  # Reduced from 0.1

# Allow deeper diving
max_depth: 50.0                 # Increased from 20.0
initial_z: -10.0                # Start deeper
```

Then rebuild and relaunch:
```bash
colcon build --packages-select ray_joystick_control
source install/setup.bash
ros2 launch ray_joystick_control joystick_launch.py
```

## Next Steps

- Explore the full [README.md](README.md) for complete documentation
- Try recording joystick commands for autonomous missions
- Configure safety limits in the control parameters
- Add haptic feedback if your controller supports it

Enjoy controlling your ROV! 🎮🤖
