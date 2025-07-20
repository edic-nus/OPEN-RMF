# ESP32-S3 Open-RMF Door Node

This project implements an Open-RMF (Robotics Middleware Framework) compatible door node using ESP32-S3-Touch-LCD-4.3 with micro-ROS Jazzy integration.

## Features

- **RMF Door Simulation**: Complete door state machine with OPEN/CLOSED/MOVING/ERROR states
- **ROS 2 Integration**: Compatible with Open-RMF fleet management systems
- **Hardware Simulation**: Physical pins simulate door motor, sensors, and locks
- **Real-time Status**: Publishes door status and state at 1Hz
- **Command Interface**: Accepts OPEN/CLOSE commands via ROS topics
- **Visual Feedback**: LED status indicators and display logging
- **Request Tracking**: Unique request IDs for door operations

## ROS 2 Topics

### Subscribed Topics:
- `/door_request` (std_msgs/String): Door commands ("OPEN", "CLOSE")

### Published Topics:
- `/door_status` (std_msgs/String): JSON status with door state and request ID
- `/door_state` (std_msgs/String): Simple state string ("OPEN", "CLOSED", "MOVING", "ERROR")

## Door States

- **CLOSED**: Door is closed and locked
- **OPEN**: Door is open and unlocked  
- **MOVING**: Door is transitioning between states (3 second duration)
- **ERROR**: Door encountered an error condition

## Hardware Simulation

- **Pin 4**: Door motor control (HIGH = running, LOW = stopped)
- **Pin 5**: Door open sensor (INPUT_PULLUP)
- **Pin 6**: Door closed sensor (INPUT_PULLUP)
- **Pin 7**: Door lock control (HIGH = locked, LOW = unlocked)
- **Pin 2**: Status LED (steady = open, off = closed, blinking = moving/error)
- **Pin 38**: Display backlight control

## Setup Instructions

### 1. Prerequisites

Make sure you have ROS 2 Jazzy installed on your host computer with micro-ROS:

```bash
# Install micro-ROS agent
sudo apt update
sudo apt install ros-jazzy-micro-ros-agent
```

### 2. Build and Upload

1. Open this project in VS Code with PlatformIO extension
2. Build the project (Ctrl+Shift+P → "PlatformIO: Build")
3. Upload to your ESP32-S3 board (Ctrl+Shift+P → "PlatformIO: Upload")

### 3. Run micro-ROS Agent

On your host computer, run the micro-ROS agent:

```bash
# Option 1: Use the provided script
./launch_microros_agent.sh

# Option 2: Run directly
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

### 4. Test Communication

In separate terminals, you can test the communication:

```bash
# Listen to the counter topic
ros2 topic echo /esp32/counter

# Send commands to the ESP32
ros2 topic pub /esp32/command std_msgs/String "data: 'Hello ESP32!'"

# List all topics
ros2 topic list
```

## Troubleshooting

1. **No serial communication**: 
   - Check that `/dev/ttyACM0` is the correct port
   - Ensure user is in `dialout` group: `sudo usermod -a -G dialout $USER`
   - Try resetting the ESP32 board

2. **micro-ROS agent connection fails**:
   - Make sure the ESP32 is fully booted (wait ~3 seconds after upload)
   - Check serial monitor for "micro-ROS setup complete!" message
   - Verify baud rate matches (115200)

3. **Build errors**:
   - Clean and rebuild: PlatformIO → Clean → Build
   - Check internet connection (micro-ROS library downloads from GitHub)

## LED Status Indicators

- **Solid ON during setup**: micro-ROS initializing
- **OFF after setup**: Ready and running
- **Brief blink**: Message received on `/esp32/command` topic
- **Fast blinking**: Error state (check serial monitor)

## Topics

- `/esp32/counter` (std_msgs/Int32): Published every second with incrementing counter
- `/esp32/command` (std_msgs/String): Subscribe to receive commands that trigger LED blink
