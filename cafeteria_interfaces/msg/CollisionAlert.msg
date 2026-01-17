# CollisionAlert.msg
# Alert message published when a collision is detected
# Published by: IMU or Bumper sensor nodes
# Subscribed by: Robot State Machine

# Collision type constants
uint8 TYPE_BUMPER=0       # Physical bumper contact
uint8 TYPE_IMU=1          # IMU detected sudden deceleration/impact

# Type of collision detected
uint8 collision_type

# Direction of impact (normalized vector)
geometry_msgs/Vector3 impact_direction

# Severity level (0-100, higher = more severe)
uint8 severity

# Timestamp of collision
builtin_interfaces/Time stamp
