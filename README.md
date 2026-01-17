# ITU Cafeteria Bot

A ROS2 Humble simulation of a TurtleBot3 robot navigating an ITU Medical Faculty cafeteria environment using Gazebo. Features an intelligent **Finite State Machine (FSM)** for autonomous food delivery with theft prevention and operator intervention capabilities.

## 🚀 Features

- **Autonomous Navigation** - Nav2 integration for table-to-table delivery
- **Theft Prevention** - 30-second grace period when food is lifted
- **Collision Detection** - IMU/Bumper triggered emergency stops
- **Operator Interface** - Interactive CLI for human intervention
- **Recovery Behaviors** - Automatic search patterns for lost markers

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
sudo apt install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-description ros-humble-nav2-msgs ros-humble-nav2-bringup -y
```

### 3. Build All Packages

```bash
cd ~/itu_cafeteria_bot
colcon build --packages-select cafeteria_interfaces cafeteria_robot_fsm cafeteria_simulation
source install/setup.bash
```

### 4. Launch Gazebo Simulation

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch cafeteria_simulation gazebo_world.launch.py
```

### 5. Run the Robot State Machine

In a new terminal:

```bash
source install/setup.bash
ros2 run cafeteria_robot_fsm robot_state_machine
```

### 6. Control with Operator Panel

In another terminal:

```bash
source install/setup.bash
ros2 run cafeteria_robot_fsm operator_panel
```

## Project Structure

```
itu_cafeteria_bot/
├── README.md
├── IMPLEMENTATION_PLAN.md      # Technical design document
├── WALKTHROUGH.md              # Implementation summary
├── docker-compose.yml
│
├── cafeteria_simulation/       # Gazebo simulation package
│   ├── launch/
│   │   └── gazebo_world.launch.py
│   └── worlds/
│       └── med_cafeteria_mapping.world
│
├── cafeteria_interfaces/       # Custom ROS2 messages/services
│   ├── msg/
│   │   ├── FoodStatus.msg      # FSR sensor data
│   │   ├── CollisionAlert.msg  # IMU/Bumper alerts
│   │   └── RobotState.msg      # FSM state broadcast
│   └── srv/
│       ├── OperatorCommand.srv # Operator controls
│       └── StartDelivery.srv   # Delivery requests
│
└── cafeteria_robot_fsm/        # State Machine package
    ├── cafeteria_robot_fsm/
    │   ├── robot_state_machine.py  # Main FSM (~800 lines)
    │   └── operator_panel.py       # Interactive CLI
    └── launch/
        ├── master.launch.py    # Full system launch
        └── fsm_only.launch.py  # FSM-only launch
```

## Robot States

| State | Description |
|-------|-------------|
| 🟢 IDLE | Waiting for delivery command |
| 🚀 NAVIGATING | Traveling to target table |
| ⚠️ THEFT_GRACE | Food lifted - 30s grace period |
| 🛑 EMERGENCY_STOP | Collision detected - awaiting operator |
| 🔍 RECOVERY_SEARCH | Searching for lost marker |
| 📍 ARRIVED | At destination - waiting for pickup |
| 🏠 RETURN_WITH_FOOD | Returning home after timeout |

## Operator Panel Commands

```
status      - Show current robot state
resume      - Resume from emergency stop
abort       - Abort mission, return to IDLE
theft       - Confirm theft incident
return      - Force immediate return to home
deliver <n> - Start delivery to table n (1-5)
help        - Show all commands
quit        - Exit panel
```

## ROS2 Topics & Services

**Topics:**
- `/food_status` - FoodStatus (FSR sensor → FSM)
- `/collision_alert` - CollisionAlert (IMU/Bumper → FSM)
- `/robot_state` - RobotState (FSM → UI/logging)
- `/cmd_vel` - Twist (FSM → motors)

**Services:**
- `/operator_command` - Resume/Abort/Theft commands
- `/start_delivery` - Start new delivery mission

**Actions:**
- `/navigate_to_pose` - Nav2 navigation

## Useful Docker Commands

```bash
docker compose up -d          # Start container
docker compose exec ros-gazebo bash  # Attach to container
docker compose down           # Stop container
```

## X Server Setup (Windows)

1. Download and install [VcXsrv](https://sourceforge.net/projects/vcxsrv/)
2. Launch XLaunch with:
   - Multiple windows
   - Start no client
   - **Disable access control** (important!)
3. The Gazebo GUI will display on your Windows desktop

## Documentation

- [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) - Technical design and architecture
- [WALKTHROUGH.md](WALKTHROUGH.md) - What was built and how to test

## License

MIT License
