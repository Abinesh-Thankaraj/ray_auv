# PS5 Joystick Control - Visual Reference Guide

## PS5 DualSense Controller Button & Axis Layout

```
                         ╔═══════════════════════╗
                         ║                       ║
                         ║   PS5 DualSense       ║
                         ║   Controller          ║
                         ║                       ║
          ╔═════════════════════════════════════════════════╗
          ║                                                 ║
          ║  ╔─────────────────────────────────────────╗   ║
          ║  │          △ (Triangle) - Button 3        │   ║
          ║  │         / \                             │   ║
          ║  │        /   \                            │   ║
          ║  │  □ (Square) ○ (Circle)                  │   ║
          ║  │   Button 0   Button 2                   │   ║
          ║  │        \   /                            │   ║
          ║  │         \ /                             │   ║
          ║  │      ✕ (Cross) - Button 1               │   ║
          ║  │     (MODE TOGGLE)                       │   ║
          ║  └─────────────────────────────────────────┘   ║
          ║                                                 ║
          ║  L1 (Btn 4)              R1 (Btn 5)            ║
          ║  ┌──────────┐            ┌──────────┐          ║
          ║  │ L2 Ax2   │            │ R2 Ax5   │          ║
          ║  │ (Button) │            │ (Button) │          ║
          ║  │          │            │          │          ║
          ║  │ DESCEND  │            │ ASCEND   │          ║
          ║  │ (Depth-) │            │ (Depth+) │          ║
          ║  └──────────┘            └──────────┘          ║
          ║                                                 ║
          ║  ╔───────┐              ╔───────┐              ║
          ║  │ Left  │              │ Right │              ║
          ║  │ Stick │              │ Stick │              ║
          ║  │       │              │       │              ║
          ║  │   ⬆   │              │   ⬆   │              ║
          ║  │ ⬅ ⊙ ➡ │              │ ⬅ ⊙ ➡ │              ║
          ║  │   ⬇   │              │   ⬇   │              ║
          ║  │       │              │       │              ║
          ║  │ Ax0/1 │              │ Ax3/4 │              ║
          ║  │  XY   │              │  XY   │              ║
          ║  │Motion │              │Rotation              ║
          ║  │ Strafe │             │Click Btn9│           ║
          ║  │Forward │             │          │           ║
          ║  │ Lateral │            │          │           ║
          ║  │        │              │          │           ║
          ║  └───────┘              └───────┘              ║
          ║                                                 ║
          ║              [ Touchpad ]                       ║
          ║            (Click = Btn 11)                    ║
          ║                                                 ║
          ║          [ PS Button ] (Btn 10)                ║
          ║                                                 ║
          ║            [ Mic Button ] (Btn 12)             ║
          ║                                                 ║
          └─────────────────────────────────────────────────┘
```

## Motion Control Mapping

### Basic 6-DOF Movement

```
                        PITCH UP (↑)
                              │
                              │
             ROLL LEFT ←─── [Right Stick] ──→ ROLL RIGHT
                              │
                              │
                        PITCH DOWN (↓)


          STRAFE LEFT        FORWARD
                ↖              ↑
                  \            │
                    \          │
        ←─────── [Left Stick] ─────→  STRAFE RIGHT
                    /          │
                  /            │
                ↙              ↓
                        BACKWARD

        L2 TRIGGER ────→ DESCEND (Depth -)
        R2 TRIGGER ────→ ASCEND  (Depth +)
```

### Yaw Control (Press X to Toggle)

```
Normal Mode (Default):
┌─────────────────────────────────────┐
│  RIGHT STICK controls:              │
│  • X-axis → ROLL Left/Right         │
│  • Y-axis → PITCH Up/Down           │
│  Press X button to enable Yaw Mode  │
└─────────────────────────────────────┘

Yaw Mode (After pressing X):
┌─────────────────────────────────────┐
│  RIGHT STICK controls:              │
│  • X-axis → YAW Left/Right ◄ NEW    │
│  • Y-axis → PITCH Up/Down (same)    │
│  Press X button again to return     │
└─────────────────────────────────────┘
```

## Complete Control Flow

```
                    PS5 DualSense
                    Controller
                         │
                         ├──→ Left Stick X/Y
                         ├──→ Right Stick X/Y
                         ├──→ L2/R2 Triggers
                         ├──→ X Button
                         └──→ Other Buttons
                         
                         │
                    [joy_node]
                    Converts to:
                    sensor_msgs/Joy
                         │
                         ▼
                    /ray/joy topic
                         │
                         ▼
            [joystick_controller.py]
            Processes input and
            integrates position
                         │
                         ├──→ Apply deadzone
                         ├──→ Scale by velocity
                         ├──→ Convert Euler → Quaternion
                         ├──→ Clamp depth
                         └──→ Update pose state
                         
                         ▼
                    /ray/cmd_pose
                    (PoseStamped)
                         │
                         ▼
            [body_control_sm]
            (auv_control sliding mode)
            Calculates control forces
                         │
                         ▼
                    /ray/wrench
                         │
                         ▼
            [ray_thruster_manager]
            Allocates to 6 thrusters
                         │
                         ├──→ thruster1
                         ├──→ thruster2
                         ├──→ thruster3
                         ├──→ thruster4
                         ├──→ thruster5
                         └──→ thruster6
                         
                         ▼
                    GAZEBO SIM
                    (or Real Thrusters)
                         │
                         ▼
                    RAY AUV MOVES! 🤖
```

## Axis Number Reference

```
Joystick Input (sensor_msgs/Joy)
├── axes[0]:  Left Stick X (LATERAL: -1=LEFT, +1=RIGHT)
├── axes[1]:  Left Stick Y (FWD/BWD: -1=FORWARD, +1=BACKWARD)
├── axes[2]:  L2 Trigger   (TRIGGER: -1=NOT, +1=FULL)
├── axes[3]:  Right Stick X (ROLL/YAW: -1=LEFT, +1=RIGHT)
├── axes[4]:  Right Stick Y (PITCH: -1=UP, +1=DOWN)
├── axes[5]:  R2 Trigger    (TRIGGER: -1=NOT, +1=FULL)
│
└── buttons[1]:  X Button (MODE TOGGLE)
```

## Depth Control Visualization

```
R2 Trigger (Ascend)
      ↑
      │  0.0 m ═══════════ Water Surface
      │  ↑
      │  -5.0 m ─────── Initial Position
      │  ↑
      │  -10.0 m
      │  ↑
      │  -20.0 m ════════ Max Depth (configurable)
      │
L2 Trigger (Descend)
```

## Parameter Tuning Guide

```
DEADZONE Effect:
Small (0.05) ──→ More responsive, may drift
Large (0.15) ──→ Less drift, slower response
     ▲
     │  ╔════════════════════╗
     │  ║ Sweet Spot: 0.08   ║
     │  ║ to 0.12            ║
     │  ╚════════════════════╝
     │ /
Responsiveness

MAX_ANGULAR_VELOCITY:
0.78 rad/s  ≈ 45°/s  (Slow)
1.57 rad/s  ≈ 90°/s  (Medium - Default)
3.14 rad/s  ≈ 180°/s (Fast)

MAX_LINEAR_VELOCITY:
0.5 m/s (Turtle Mode)
1.0 m/s (Balanced - Default)
2.0 m/s (Speed Demon)
```

## Control Combinations

### Forward & Descend
```
    ↑
    │ (Left Stick Y Forward)
    │
    └─────→ + Press L2 (Descend)
```

### Strafe While Pitching
```
←─────→ (Left Stick X)
         +
         ↓ (Right Stick Y)
```

### Yaw While Moving
```
→ (Left Stick Y)  +  X Button + ← (Right Stick X)
Forward            Yaw Mode      Yaw Left
```

## Status Indicators

```
Terminal Output Signs:

✓ "Joystick Controller Initialized"
  ├─→ Node is running
  ├─→ Listening for /joy messages
  └─→ Publishing to /cmd_pose

✓ "Switched to YAW mode"
  ├─→ X Button was pressed
  ├─→ Right Stick X now controls Yaw
  └─→ Right Stick Y still controls Pitch

✓ Joy message received (ros2 topic echo /joy)
  ├─→ Controller is communicating
  ├─→ Raw data from PS5 is flowing
  └─→ No deadzone applied yet

✓ Pose message published (ros2 topic echo /cmd_pose)
  ├─→ Controller is processing input
  ├─→ Position/orientation is being updated
  └─→ Ready to control ROV
```

## Troubleshooting Visual

```
Problem Tree:

    Joystick Not Working?
            │
    ┌───────┴───────┐
    ▼               ▼
No /joy      No /cmd_pose
message      message
    │               │
    ├─ Is joy_node  ├─ Is joystick
    │  running?     │  controller
    │               │  running?
    ├─ Is                ├─ Check
    │  controller         │  parameters
    │  connected?         │  loaded?
    │               │
    └─ Check        └─ Check
       /dev/input     ROS logs
```

## Emergency Controls

```
In case of emergency:

  STOP JOYSTICK INPUT:
  Neutral all sticks and triggers to center/zero

  EMERGENCY STOP:
  Power off PS5 controller or press PS button

  EMERGENCY DEPTH:
  Rapidly press R2 trigger to ascend quickly

  Manual Recovery:
  Kill nodes: Ctrl+C on launch terminal
  Check status: ros2 node list
```

---

This visual reference helps you understand the complete control system and how each input maps to ROV motion.
