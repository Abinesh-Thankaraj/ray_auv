# Custom Thruster Manager

This package provides a custom thruster allocation matrix (TAM) configuration for the Ray AUV that implements a user-defined thruster control scheme.

## Thruster Allocation Scheme

The custom TAM implements the following thruster control scheme:

- **Thruster 1**: Primarily responsible for sway motion (Y-axis movement)
- **Thruster 2, 3**: Primarily responsible for surge motion (X-axis movement) and yaw rotation
- **Thruster 4, 5, 6**: Primarily responsible for heave motion (Z-axis movement), pitch rotation, and roll rotation

## TAM Matrix Configuration

The custom TAM matrix is configured as follows:

```
[Surge]   [ 0  1  1  0  0  0 ] [Thruster1]
[Sway]    [ 1  0  0  0  0  0 ] [Thruster2]
[Heave]   [ 0  0  0  1  1  1 ] [Thruster3]
[Roll]    [ 0  0  0  0.3 -0.3 -0.3] [Thruster4]
[Pitch]   [ 0  0  0  0.5  0.5  0.5 ] [Thruster5]
[Yaw]     [ 0  0.5 -0.5 0  0  0 ] [Thruster6]
```

## Usage

### Building the Package

```bash
cd /home/abinesh/ray_ws
colcon build --packages-select custom_thruster_manager
source install/setup.bash
```

### Running with Custom Thruster Manager

To use the custom thruster manager instead of the default one, run:

```bash
ros2 launch custom_thruster_manager custom_sliding_mode_launch.py
```

This will:
1. Load the custom thruster manager with the user-defined TAM
2. Load the sliding mode controller
3. Start the slider publishers for pose control
4. Launch RViz for visualization

### Slider Control

The slider UI allows you to control:
- **X, Y, Z**: Position control (surge, sway, heave)
- **Roll, Pitch, Yaw**: Orientation control

The custom thruster manager will automatically allocate the appropriate thruster forces to achieve the desired pose.

## Parameters

The custom thruster manager can be configured with the following parameters:

- `min_thrust`: Minimum thrust value (default: -40.0)
- `max_thrust`: Maximum thrust value (default: 40.0)
- `deadzone`: Deadzone for thruster commands (default: 1.0)
- `thruster_names`: Array of thruster joint names

## Customization

To modify the TAM configuration, edit the `setupCustomTAM()` function in `src/custom_thruster_manager.cpp`. The TAM matrix defines how each thruster contributes to the 6-DOF wrench (surge, sway, heave, roll, pitch, yaw).

## Integration with Ray

The custom thruster manager integrates seamlessly with the existing Ray control system. It subscribes to wrench commands from the sliding mode controller and publishes thruster commands that are compatible with the Gazebo thruster plugins.
