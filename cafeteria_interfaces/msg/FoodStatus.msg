# FoodStatus.msg
# Reports the current status of food on the tray
# Published by: FSR (Force Sensing Resistor) sensor node
# Subscribed by: Robot State Machine

# Status constants
uint8 STATUS_PRESENT=0    # Food is on the tray (normal weight detected)
uint8 STATUS_LIFTED=1     # Food has been lifted (weight reduced but not zero)
uint8 STATUS_REMOVED=2    # Food has been completely removed

# Current status
uint8 status

# Measured weight in kilograms
float32 weight_kg

# Timestamp of the measurement
builtin_interfaces/Time stamp
