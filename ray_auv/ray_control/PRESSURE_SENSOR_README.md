# Underwater Pressure Sensor System

This document explains the enhanced pressure sensor system for the Ray AUV that provides depth-based pressure readings with proper unit conversions.

## Overview

The pressure sensor system has been upgraded to:
- **Respond to actual depth changes** in the underwater environment
- **Convert pressure readings** from kPa to bar units
- **Calculate depth in meters** below the water surface
- **Provide realistic pressure values** that increase with depth (1 bar per 10 meters)

## Components

### 1. Underwater Pressure Plugin
- **File**: `sensors.xacro`
- **Plugin**: `libuuv_gazebo_ros_subsea_pressure_plugin.so`
- **Configuration**:
  - Atmospheric pressure: 101.325 kPa (sea level)
  - Pressure increase: 9.80638 kPa per meter depth
  - Update rate: 20 Hz
  - Noise: 0.1 kPa standard deviation

### 2. Pressure Converter Node
- **File**: `scripts/pressure_converter.py`
- **Function**: Converts pressure readings and calculates depth
- **Topics**:
  - **Input**: `ray/pressure` (sensor_msgs/FluidPressure)
  - **Output**: `ray/pressure_bar` (std_msgs/Float64) - pressure in bar
  - **Output**: `ray/depth_meters` (std_msgs/Float64) - depth in meters

### 3. Test Node
- **File**: `scripts/test_pressure_sensor.py`
- **Function**: Monitors and displays pressure readings
- **Usage**: Run to verify pressure sensor is working correctly

## Pressure Calculations

### Standard Conversions
- **1 bar = 100 kPa**
- **Depth (m) = (Total Pressure - Atmospheric Pressure) / (Water Density × Gravity)**
- **For seawater**: ~1 bar pressure increase per 10 meters depth

### Example Values
| Depth (m) | Pressure (kPa) | Pressure (bar) |
|-----------|----------------|----------------|
| 0         | 101.325        | 1.013          |
| 10        | 199.388        | 1.994          |
| 20        | 297.451        | 2.975          |
| 50        | 591.644        | 5.916          |
| 100       | 1081.963       | 10.820         |

## Usage

### 1. Launch with Pressure Sensor
```bash
# Source the workspace
source /home/pdc/ray_ws/install/setup.bash

# Launch with pressure converter (any of these):
ros2 launch ray_control custom_sliding_mode_launch.py
ros2 launch ray_control sliding_mode_launch.py
ros2 launch ray_control cascaded_pids_launch.py
```

### 2. Monitor Pressure Readings
```bash
# Monitor raw pressure (kPa)
ros2 topic echo /ray/pressure

# Monitor pressure in bar
ros2 topic echo /ray/pressure_bar

# Monitor depth in meters
ros2 topic echo /ray/depth_meters

# Run test node for continuous monitoring
ros2 run ray_control test_pressure_sensor.py
```

### 3. Test Depth Changes
1. Launch the simulation with underwater world
2. Move the Ray model deeper using the control interface
3. Observe pressure readings increase with depth
4. Verify that pressure increases by approximately 1 bar per 10 meters

## Topics

### Input Topics
- `/ray/pressure` (sensor_msgs/FluidPressure)
  - Raw pressure reading from the underwater pressure plugin
  - Units: kPa (kilopascals)

### Output Topics
- `/ray/pressure_bar` (std_msgs/Float64)
  - Pressure converted to bar units
  - Conversion: bar = kPa / 100

- `/ray/depth_meters` (std_msgs/Float64)
  - Calculated depth below water surface
  - Units: meters
  - Formula: depth = (pressure - atmospheric) / (density × gravity)

## Configuration

### Pressure Converter Parameters
- `pressure_topic`: Input pressure topic (default: 'pressure')
- `bar_topic`: Output bar pressure topic (default: 'pressure_bar')
- `depth_topic`: Output depth topic (default: 'depth_meters')
- `atmospheric_pressure_kpa`: Atmospheric pressure at sea level (default: 101.325)
- `water_density`: Water density in kg/m³ (default: 1025.0 for seawater)
- `gravity`: Gravitational acceleration (default: 9.80665 m/s²)

### Underwater Pressure Plugin Parameters
- `standard_pressure`: Atmospheric pressure at sea level (101.325 kPa)
- `kPa_per_meter`: Pressure increase per meter depth (9.80638 kPa/m)
- `estimate_depth_on`: Enable depth estimation (true)
- `update_rate`: Sensor update rate (20 Hz)
- `saturation`: Maximum pressure range (30000 kPa)

## Troubleshooting

### No Pressure Data
1. Check if the underwater world is properly loaded
2. Verify the pressure plugin is active in the world file
3. Ensure the Ray model is in the water (not above surface)

### Incorrect Pressure Values
1. Verify the Ray model is actually underwater
2. Check that the pressure sensor link is properly attached
3. Ensure the underwater world has proper physics simulation

### Topics Not Publishing
1. Check if the pressure converter node is running
2. Verify topic names match between sensor and converter
3. Check ROS2 node list: `ros2 node list`

## Technical Details

### Physics Simulation
The pressure sensor uses the UUV Gazebo pressure plugin which:
- Calculates pressure based on the link's Z-position in the world
- Applies realistic hydrostatic pressure calculations
- Includes configurable noise and saturation limits

### Coordinate System
- Z-axis positive direction is upward
- Depth is calculated as negative Z-position
- Pressure increases as Z-position decreases (going deeper)

### Performance
- Update rate: 20 Hz
- Low computational overhead
- Suitable for real-time control applications

