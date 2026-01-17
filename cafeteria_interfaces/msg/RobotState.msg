# RobotState.msg
# Publishes current state of the robot state machine
# Published by: Robot State Machine
# Subscribed by: Operator UI, Logging systems

# State constants (matching RobotState enum in FSM)
uint8 STATE_IDLE=0
uint8 STATE_NAVIGATING=1
uint8 STATE_THEFT_GRACE=2
uint8 STATE_EMERGENCY_STOP=3
uint8 STATE_RECOVERY_SEARCH=4
uint8 STATE_ARRIVED=5
uint8 STATE_RETURN_WITH_FOOD=6

# Current state
uint8 current_state

# Previous state (for debugging/logging)
uint8 previous_state

# Human-readable state name
string state_name

# Time spent in current state (seconds)
float32 time_in_state

# Additional status message
string status_message
