"""Values shared by all nodes: turtlesim defaults and the limits of the window."""

# Where turtlesim puts the first turtle (centre of the window, facing +x).
START_X = 5.544445
START_Y = 5.544445
START_YAW = 0.0

# The turtlesim window is 11 x 11 units.
WORLD_MIN = 0.0
WORLD_MAX = 11.0

# Square action: default side and biggest side that keeps the square inside the window
# (the square grows to the right and up from the start pose, with a 0.5 margin).
DEFAULT_SIDE = 3.0
MAX_SIDE = WORLD_MAX - 0.5 - START_X

# Default values of the ROS parameters.
DEFAULT_MAX_LINEAR_SPEED = 2.0    # units/s
DEFAULT_MAX_ANGULAR_SPEED = 2.0   # rad/s
DEFAULT_POSITION_TOLERANCE = 0.05  # units
DEFAULT_ANGLE_TOLERANCE = 0.02     # rad
MAX_ALLOWED_SPEED = 10.0           # sanity limit for the speed parameters

# Default pen of turtlesim (used by the reset service to turn the pen back on).
PEN_R, PEN_G, PEN_B, PEN_WIDTH = 179, 184, 255, 3

# Topic / service names of turtlesim.
CMD_VEL_TOPIC = '/turtle1/cmd_vel'
POSE_TOPIC = '/turtle1/pose'
CLEAR_SERVICE = '/clear'
TELEPORT_SERVICE = '/turtle1/teleport_absolute'
SET_PEN_SERVICE = '/turtle1/set_pen'

# Control loop.
LOOP_PERIOD = 0.02  # 50 Hz
GOAL_TIMEOUT = 60.0  # seconds before a goToPose goal is aborted
