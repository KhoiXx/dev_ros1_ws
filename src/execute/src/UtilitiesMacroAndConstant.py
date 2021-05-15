import math

ROS_MODE = True

# Vehicle sufficient macros
VEL_SCALE = 6
#  centimet
ROBOT_WIDTH = 45
ROBOT_HEIGHT = 32
# ROBOT_LENGTH = 48
# ROBOT_WIDTH = 30
# GM6020
ROBOT_LENGTH = 45
ROBOT_WIDTH = 32
# GM6020
ROBOT_WHEEL_RADIUS = 7.8
PULSE_ENCODER_TO_METER = 11264.0
# 1 pixel is 2cm
MAP_RESOLUTION = 2
#  centimet
ROBOT_ANGLE = math.atan(ROBOT_HEIGHT / ROBOT_WIDTH)
ROBOT_DISTANCE_FROM_CENTER = math.sqrt((ROBOT_HEIGHT / 2) * (ROBOT_HEIGHT / 2) + (ROBOT_WIDTH / 2) * (ROBOT_WIDTH / 2))
SCALE = 10
ROBOT_SCALE = 5
# centimet
CENTER_X = 0
# centimet
CENTER_Y = 0
# millimeter
DISTANCE_BETWWEN_LED = 16
# millimeter
ROBOT_DISTANCE_FROM_CENTER_TO_SENSOR = 240
# km/h
ROBOT_MAX_SPEED = 7.92

# Initailize speed and angle value
front_left_angle = 0
front_right_angle = 0
rear_left_angle = 0
rear_right_angle = 0

front_left_speed = 0
front_right_speed = 0
rear_left_speed = 0
rear_right_speed = 0

# Serial parameters
BAUDRATE = 115200
PORT = "/dev/ttyTHS1"
START_FRAME = "S"
END_FRAME = "E"
LENGTH_OF_DATA_RECEIVE = 22

# PID PARAMETERS
Kp = 1.8
Kd = 2
Ki = 30

# NAV PARAMETERS
ENCODER_TO_METER = 11264
DISTANCE_THRESH = 11264
NAV_DEBUG = False
SPIN_ANGLE_STEP = 1

# MOTOR PARAMETER
MAX_ENCODER_PULSE = 4294967296