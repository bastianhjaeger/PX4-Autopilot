# Actuator angle setpoint message in degree up to 8 actuators.
# The setpoint is in degrees and should be used when a normalized setpoint is insufficient
# and the real value of angle matters for the application.

uint64 timestamp                      # [us] Time since system start

uint8 NUM_CONTROLS = 8
float32[8] angle_setpoint             # [deg] [@range -180, 180] Actuator angle setpoints.

# TOPICS steering_angle_setpoint
