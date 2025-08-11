# Lift System

This part of the project implements a lift (elevator) system that can be controlled by Turtlebot3 through MQTT communication. 

---

## Structure

- `/desktop`  
  Contains the code for performing MQTT communication to the lift system through your desktop

- `/nav`  
  Contains the code for controlling Turtlebot for navigation and performing MQTT communication to the lift system

- `/rpi`  
  Contains the code for initializing an MQTT listener on the Raspberry Pi that controls the lift.

---

## Setup Instructions

### 1. Clone the Repository

```bash
git clone https://github.com/edic-nus/OPEN-RMF.git
cd OPEN-RMF
cd lift
```

### 2. Choose your Scenario

#### Scenario 1: Turtlebot to Lift Communication

1. Connect to Turtlebot Raspberry Pi

```bash
ssh <username>@<ip-address>
```

2. Start Core Components (in the Turtlebot Raspberry Pi)

```bash
rosbu
```

3. Connect to Lift Raspberry Pi 

```bash
ssh <username>@<ip-address>
```

4. Run the MQTT Listener (in the Lift Raspberry Pi)

```bash
cd rpi
python mqtt_listener.py
```

5. Run the Turtlebot Controller (in your computer)

```bash
cd nav
python manual_nav.py
```

After doing these steps, you should be able to control your Turtlebot and perform MQTT communication to the Lift Raspberry Pi by sending messages to the terminal. 

#### Scenario 2: Desktop to Lift Communication

1. Connect to Lift Raspberry Pi 

```bash
ssh <username>@<ip-address>
```

2. Run the MQTT Listener (in the Lift Raspberry Pi)

```bash
cd rpi
python mqtt_listener.py
```

3. Run the lift controller (in your computer) 

```bash
cd desktop
sudo python mqtt_publisher.py
```

After doing these steps, you should be able to control the Lift Raspberry Pi by sending messages to the terminal.