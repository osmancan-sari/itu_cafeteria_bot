# ITU Cafeteria Bot

A ROS2 Humble simulation of a TurtleBot3 robot navigating an ITU Medical Faculty cafeteria environment using Gazebo.

## Prerequisites

- **Docker Desktop** for Windows
- **X Server** (VcXsrv or Xming) for GUI display
- Docker image: `ros-gazebo:humble`

## Quick Start

### 1. Start the Docker Container

```bash
cd itu_cafeteria_bot
docker compose up -d
docker compose exec ros-gazebo bash
```

### 2. Install Dependencies (First Time Only)

Inside the container:

```bash
sudo apt update
sudo apt install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-description ros-humble-nav2-msgs -y
```

### 3. Build the Project

```bash
cd ~/itu_cafeteria_bot
colcon build --packages-select cafeteria_simulation
source install/setup.bash
```

### 4. Launch Gazebo Simulation

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch cafeteria_simulation gazebo_world.launch.py
```

## Project Structure

```
itu_cafeteria_bot/
├── docker-compose.yml          # Docker configuration
├── README.md                   # This file
└── cafeteria_simulation/       # ROS2 package
    ├── CMakeLists.txt          # Build configuration
    ├── package.xml             # Package dependencies
    ├── launch/
    │   └── gazebo_world.launch.py   # Launch file for Gazebo
    └── worlds/
        ├── med_cafeteria.world           # Cafeteria world file
        └── med_cafeteria_mapping.world   # Cafeteria world for mapping
```

## Useful Commands

```bash
# Start container
docker compose up -d

# Attach to running container
docker compose exec ros-gazebo bash

# Open additional terminal in container
docker compose exec ros-gazebo bash

# Stop container
docker compose down
```

## X Server Setup (Windows)

1. Download and install [VcXsrv](https://sourceforge.net/projects/vcxsrv/)
2. Launch XLaunch with these settings:
   - Multiple windows
   - Start no client
   - **Disable access control** (important!)
3. The Gazebo GUI will display on your Windows desktop

## License

TODO: Add license
