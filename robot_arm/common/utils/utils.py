DEVICENAME = 'COM1'
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0
DXL_IDS = [1, 2, 3, 4, 5, 6]
DXL_MIN_POS = 0
DXL_MAX_POS = 4095
DXL_MOVING_STATUS_THRESHOLD = 100
DXL_STALL_LOAD_THRESHOLD = 400
DXL_STALL_DISTANCE_THRESHOLD = 10
DXL_VELOCITY_STALL_THRESHOLD = 5

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_OPERATING_MODE = 11
ADDR_MOVING_SPEED = 104
ADDR_VELOCITY_LIMIT = 44
ADDR_PROFILE_VELOCITY = 112
ADDR_PRESENT_LOAD = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_MOVING = 122

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0


def parse_limits(file_path="robot_arm/configs/arm/motor_limits.txt"):
    limits = {}
    try:
        with open(file_path, 'r') as f:
            for line in f:
                if line.startswith("motor_id"):
                    continue  # Skip header line
                id_str, min_pos_str, max_pos_str = line.strip().split(',')
                limits[int(id_str)] = (int(min_pos_str), int(max_pos_str))
    except FileNotFoundError:
        print(f"Limits file {file_path} not found. Using default limits.")
    return limits


def parse_home(file_path="robot_arm/configs/arm/home_positions.txt"):
    home_positions = {}
    try:
        with open(file_path, 'r') as f:
            for line in f:
                id_str, position_str = line.strip().split(',')
                home_positions[int(id_str)] = int(position_str)
    except FileNotFoundError:
        print(
            f"Home positions file {file_path} not found. Using default home positions.")
    return home_positions
