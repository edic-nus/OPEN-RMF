# Lift Control Package

A ROS2 package for controlling a two-level lift system using action servers and clients. To be noted is that this lift_control package is meant to be built on the Raspberry Pi for the lift. Attempts to build the package on remote PC would likely fail due to the dependency on the library WiringPi for GPIO control. However, if building the lift_control package on remote PC (not Raspberry Pi), delete lift_rpi.cpp and its mentions from package.xml and CMakeLists.txt

A Docker configuration is also provided in the /Docker folder for other hardware configurations.

## Overview

This package implements a state-based lift control system with the following components:
- **Lift State Server**: Manages lift position and movement
- **Lift Commander**: Command-line interface for controlling the lift
- **Custom Action Interface**: Defines commands, feedback, and results

## Package Structure
```
lift_control/
├── action/
│   └── Command.action          # Action definition
├── src/
│   ├── lift_state.cpp          # Lift server node (Simulation for testing: No hardware integration)
│   └── lift_command.cpp        # Lift client node (Action Client for testing)
│   └── lift_rpi.cpp            # Actual lift server node (Integrated with Raspberry Pi GPIO)
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Action Interface

### Command.action

**Goal (Commands):**
- `0` - STOP: Emergency stop (only allowed while moving)
- `1` - GO UP: Move to second level (from first level or between levels)
- `2` - GO DOWN: Move to first level (from second level or between levels)

**Result:**
- `completion`: 
  - `0` - Moving (partial completion)
  - `1` - Reached target level

**Feedback:**
- `status`:
  - `0` - Moving
  - `1` - Reached

## Lift States

The lift operates in five distinct states with specific command restrictions:

| State Code | Description | Allowed Commands |
|------------|-------------|------------------|
| `001` | Stationary at First Level | GO UP only |
| `100` | Stationary at Second Level | GO DOWN only |
| `010` | Stationary Between Levels | GO UP or GO DOWN |
| `011` | Moving to First Level | STOP only |
| `110` | Moving to Second Level | STOP only |

## Building the Package
```bash
# Navigate to workspace
cd ~/lift_ws

# Build the package
colcon build --packages-select lift_control

# Source the workspace
source install/setup.bash
```

## Running the System

### Terminal 1: Start the Lift Server
```bash
ros2 run lift_control lift_state
```

The server will:
- Initialize at the first level
- Publish lift status every 2 seconds on `/lift_state` topic
- Listen for action commands on `/lift_action`

### Terminal 2: Start the Command Interface
```bash
ros2 run lift_control lift_command
```

The CLI will display:
```
=== LIFT CONTROL ===
0=STOP 1=UP 2=DOWN q=QUIT

> 
```

## Usage Examples

### Example 1: Move from First to Second Level
1. Lift starts at First Level (001)
2. Enter `1` (GO UP)
3. Lift moves to Second Level (110 → 100)
4. Command succeeds

### Example 2: Invalid Command (Rejected)
1. Lift is at Second Level (100)
2. Enter `1` (GO UP) - **REJECTED**
3. Only GO DOWN is allowed from second level

## Topics

### Published
- `/lift_state` (std_msgs/msg/String)
  - Publishes current lift status every 2 seconds
  - Format: "XXX: Description"

### Subscribed
- `/lift_state` (std_msgs/msg/String) - by lift_command
  - Displays lift status in terminal

## Actions

### /lift_action (lift_control/action/Command)
- **Server**: lift_state node
- **Client**: lift_command node
- Validates commands based on current lift state
- Provides real-time feedback during movement
- Returns completion status

## State Machine Logic

The lift implements a state-based command validation system:
```
First Level (001) --[GO UP]--> Moving Up (110) --[COMPLETE]--> Second Level (100)
                                    |
                                 [STOP]
                                    |
                                    v
Second Level (100) --[GO DOWN]--> Moving Down (011) --[COMPLETE]--> First Level (001)
                                    |
                                 [STOP]
                                    |
                                    v
                        Between Levels (010) --[GO UP/GO DOWN]--> Moving...
```

## Features

### Command Validation
- Commands are validated based on current lift state
- Invalid commands are rejected before execution
- Clear feedback on rejection reasons

### Real-time Feedback
- Movement progress updates (0-100%)
- Status updates during operation
- Completion notifications

### Safety Features
- State-based command restrictions
- Emergency stop capability
- Thread-safe state management

### Concurrent Operation
- Publisher runs on timer (500ms intervals)
- Action execution in separate thread
- Non-blocking command interface

## Development Notes

### Future Enhancements
- Hall effect sensor integration for floor detection
- Speed control for movement
- Multi-floor support (beyond 2 levels)

### Simulated Behavior
Currently, the lift:
- Simulates movement over 10 steps (5 seconds at 2 Hz)
- Uses timers instead of actual sensor feedback
- Models realistic state transitions

## Dependencies

- ROS2 Humble
- rclcpp
- rclcpp_action
- rclcpp_components
- std_msgs
- action_msgs

## Troubleshooting

### "Action server not available"
- Ensure lift_state is running before lift_command
- Check that both nodes are in the same ROS2 domain

### "Goal was REJECTED"
- Check current lift state
- Verify command is valid for current state
- See state machine diagram for allowed transitions

### Build Errors
```bash
# Clean build
cd ~/lift_ws
rm -rf build install log
colcon build --packages-select lift_control
```

## License

Apache-2.0

## Author

Jason

## Version

0.0.1
