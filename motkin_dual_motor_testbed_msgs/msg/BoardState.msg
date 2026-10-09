# State of a MOTKIN board as read from its ros2_control state interfaces.
#
# The joint part follows the sensor_msgs/JointState convention: name,
# position, velocity, effort, gain_kp and gain_kd all have the same length,
# and the i-th entry of each array refers to name[i]. A joint that does not
# export one of these state interfaces (e.g. gain_kp in simulation or on a
# passive joint) reports NaN for it.

std_msgs/Header header

string[] name

float64[] position
float64[] velocity
float64[] effort
float64[] gain_kp
float64[] gain_kd

# Status of the motkin_board GPIO. All are zero if the hardware does not
# export the corresponding state interface.
uint32 clock                 # Board clock in microseconds (wraps around).
uint32 latest_command_index  # Index of the latest command received.
uint8 flags                  # Board status flags.
