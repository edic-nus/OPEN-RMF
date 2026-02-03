# TurtleBot3 Lift Teleop

A ROS 2 teleoperation package for controlling TurtleBot3 robots with lift mechanisms using keyboard input. tb3_lift_teleop requires the lift_control package to work. However, if building the lift_control package on remote PC (not Raspberry Pi), delete lift_rpi.cpp and its mentions from package.xml and CMakeLists.txt

## Features

- **Robot Movement Control**: Standard TurtleBot3 keyboard teleoperation (WASD controls)
- **Lift Control**: Integrated action client for controlling vertical lift mechanisms
- **Real-time Feedback**: Live display of lift state and status updates

## Prerequisites

- ROS 2 (Humble/Foxy or later)
- `lift_control` package with `Command` action definition
- TurtleBot3 packages
- Python 3

## Installation

```bash
cd ~/lift_ws/src
git clone <your-repo-url> tb3_lift_teleop
cd ~/lift_ws
colcon build --packages-select tb3_lift_teleop
source install/setup.bash
```

## Usage

Set your TurtleBot3 model and run the teleop node:

```bash
export TURTLEBOT3_MODEL=burger  # or waffle, waffle_pi
ros2 run tb3_lift_teleop teleop_keyboard
```

## Keyboard Controls

### Robot Movement
- **W/X**: Increase/decrease linear velocity
- **A/D**: Increase/decrease angular velocity  
- **S / Space**: Emergency stop

### Lift Control
- **Q**: Lift up
- **E**: Lift down

### Exit
- **Ctrl+C**: Quit the program

## Topics and Actions

The node interacts with:
- **Publisher**: `/cmd_vel` (geometry_msgs/Twist) - Robot velocity commands
- **Action Client**: `/lift_action` (lift_control/Command) - Lift control commands
- **Subscriber**: `/lift_state` (std_msgs/String) - Lift status updates

## Configuration

Velocity limits are automatically set based on the TurtleBot3 model:
- **Burger**: Max linear 0.22 m/s, Max angular 2.84 rad/s
- **Waffle/Waffle Pi**: Max linear 0.26 m/s, Max angular 1.82 rad/s

## Notes

- The lift action client uses **absolute topic names** (`/lift_action`, `/lift_state`) to prevent namespace conflicts
- Lift commands are non-blocking - feedback is displayed during execution
- Only one lift command can be active at a time

## License

Apache License 2.0 (original TurtleBot3 teleop code by ROBOTIS)