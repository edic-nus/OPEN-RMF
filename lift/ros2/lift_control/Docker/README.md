# ROS2 Lift Controller - Docker Development Environment

This guide provides instructions for setting up a Docker-based development environment for the ROS2 Lift Controller on a Raspberry Pi 5, with development done remotely via SSH.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Initial Setup](#initial-setup)
- [Development Container Setup](#development-container-setup)
- [Remote Development via SSH](#remote-development-via-ssh)
- [Building and Running](#building-and-running)

---

## Prerequisites
### Lift Hardware
1. Connect Raspberry Pi 5 to USB-C power
2. Connect motor driver connectors to portable power supply at 13.5V
3. Ensure 5kg of weights attached as counterweight

### On Raspberry Pi 5

1. **Install Docker:**
   ```bash
   curl -fsSL https://get.docker.com -o get-docker.sh
   sudo sh get-docker.sh
   sudo usermod -aG docker $USER
   ```
   **Important:** Log out and back in for group changes to take effect.

2. **Install Docker Compose:**
   ```bash
   sudo apt-get update
   sudo apt-get install -y docker-compose-plugin
   ```

3. **Enable GPIO Access:**
   ```bash
   sudo usermod -aG gpio $USER
   ```

4. **Verify Docker Installation:**
   ```bash
   docker --version
   docker compose version
   ```

---

## Initial Setup

### 1. Clone or Copy Your Project to Raspberry Pi

```bash
# SSH into your Raspberry Pi
ssh pi@<raspberry-pi-ip>

# Create workspace
mkdir -p ~/lift_ws
cd ~/lift_ws
```

### 2. Project Structure

Ensure your project has this structure:
```
lift_ws/
├── .devcontainer/
│   ├── devcontainer.json
│   ├── Dockerfile
│   └── docker-compose.yml
├── src/
│   └── lift_control/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── action/
│       │   └── Command.action
│       └── src/
│           ├── lift_rpi.cpp
│           ├── lift_state.cpp
│           └── lift_command.cpp
├── Dockerfile                    # Production Dockerfile
├── docker-compose.yml            # Production docker-compose
├── docker-entrypoint.sh
└── README.md                     # This file
```

### 3. Building the Container
```
cd ~/lift_ws

# Build the Docker image (first time takes 10-15 minutes)
docker compose build

# You should see output ending with "Successfully tagged..."
```

### 4.Running the Lift Controller

```bash
docker compose up
```
Press `Ctrl+C` to stop.


### Verify Operation

```bash
# Check if container is running
docker ps

# Should show a container named "lift_rpi_node"

# View logs
docker compose logs -f

# Press Ctrl+C to exit log view (container keeps running)
```

---

## Operating the Lift

### Using TurtleBot3 Teleop (if installed)

If you have the teleop package installed on the Raspberry Pi or another machine:

```bash
# On Raspberry Pi or remote machine with ROS2
export ROS_DOMAIN_ID=30
export TURTLEBOT3_MODEL=burger
ros2 run tb3_lift_teleop teleop_keyboard
```

**Keyboard Controls:**
- `q` - Move lift up
- `e` - Move lift down
- `r` - Emergency stop (stops at current position)
- `t` - Cancel current movement

### Using ROS2 Action Commands

From any machine on the same network with ROS2 installed:

```bash
# Set the domain ID to match the lift controller
export ROS_DOMAIN_ID=30

# Move lift up
ros2 action send_goal /lift_action lift_control/action/Command "{command: 1}"

# Move lift down
ros2 action send_goal /lift_action lift_control/action/Command "{command: 2}"

# Stop lift
ros2 action send_goal /lift_action lift_control/action/Command "{command: 0}"
```

### Command Reference

| Command | Value | Description |
|---------|-------|-------------|
| GO_UP | 1 | Move lift to upper level |
| GO_DOWN | 2 | Move lift to lower level |
| STOP | 0 | Stop at current position |

---

## Monitoring

### Check Lift Status

```bash
# View current status (published every 2 seconds)
export ROS_DOMAIN_ID=30
ros2 topic echo /lift_state
```

**Status Codes:**
- `001: Stationary at First Level` - At lower level (ground)
- `100: Stationary at Second Level` - At upper level
- `010: Stationary Between Levels` - Stopped between levels
- `011: Moving to First Level` - Moving down
- `110: Moving to Second Level` - Moving up

### View Container Logs

```bash
# View recent logs
docker compose logs

# Follow logs in real-time
docker compose logs -f

# View last 50 lines
docker compose logs --tail=50
```

### Check Container Status

```bash
# List running containers
docker ps

# View container resource usage
docker stats lift_rpi_node

# Check container health
docker inspect lift_rpi_node | grep Status
```

---

## Troubleshooting

### Container Won't Start

**Check logs:**
```bash
docker compose logs
```

**Common issues:**

1. **WiringPi initialization failed**
   ```bash
   # Verify GPIO access
   ls -l /dev/gpiomem
   ls -l /dev/mem
   
   # Should show: crw-rw---- with gpio group
   ```

2. **Container exits immediately**
   ```bash
   # Check for build errors
   docker compose build --no-cache
   
   # Verify docker-compose.yml exists
   ls -l docker-compose.yml
   ```

3. **Port or resource conflicts**
   ```bash
   # Stop all containers
   docker compose down
   
   # Remove old containers
   docker container prune
   
   # Restart
   docker compose up -d
   ```

### Lift Not Responding

**Check if action server is running:**
```bash
export ROS_DOMAIN_ID=30
ros2 action list
# Should show: /lift_action
```

**Check if nodes are visible:**
```bash
export ROS_DOMAIN_ID=30
ros2 node list
# Should show: /lift_state
```

**Verify network connectivity:**
```bash
# On Raspberry Pi
hostname -I
# Note the IP address

# On remote machine
ping <raspberry-pi-ip>
```

### GPIO Not Working

**Verify privileged mode:**
```bash
docker inspect lift_rpi_node | grep Privileged
# Should show: "Privileged": true
```

**Check device mounts:**
```bash
docker inspect lift_rpi_node | grep -A 10 Devices
# Should show /dev/gpiomem and /dev/mem
```

**Test GPIO inside container:**
```bash
docker exec -it lift_rpi_node gpio readall
# Should display GPIO pin layout
```

### ROS_DOMAIN_ID Mismatch

If you can't see the lift node from other machines:

```bash
# Check environment variable
docker exec -it lift_rpi_node printenv | grep ROS_DOMAIN_ID
# Should show: ROS_DOMAIN_ID=30

# On all machines accessing the lift, ensure:
export ROS_DOMAIN_ID=30
```

---

## Maintenance

### Stopping the Lift Controller

```bash
cd ~/lift_ws

# Stop gracefully
docker compose down

# Lift will decelerate and stop safely
```

### Restarting the Lift Controller

```bash
cd ~/lift_ws

# Restart
docker compose restart

# Or stop and start
docker compose down
docker compose up -d
```

### Updating the Software

```bash
cd ~/lift_ws

# Pull latest code (if using git)
git pull

# Rebuild container
docker compose build

# Restart with new image
docker compose down
docker compose up -d
```

### Viewing Resource Usage

```bash
# Real-time resource monitoring
docker stats lift_rpi_node

# Disk usage
docker system df
```

### Cleaning Up Old Images

```bash
# Remove unused images
docker image prune

# Remove all unused resources
docker system prune
```

---

## Auto-Start on Boot

To make the lift controller start automatically when the Raspberry Pi boots:

### Method 1: Using Docker Restart Policy (Already Configured)

The `docker-compose.yml` includes `restart: unless-stopped`, which means:
- Container starts automatically on boot
- Container restarts if it crashes
- Container stays stopped if manually stopped

**Enable Docker service:**
```bash
sudo systemctl enable docker
```

**Test auto-start:**
```bash
# Start the container
docker compose up -d

# Reboot
sudo reboot

# After reboot, check if running
docker ps
```