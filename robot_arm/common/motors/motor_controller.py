"""
This module contains the serial communication logic for the robot arm's motors.
It handles low-level operations such as connecting to the motors, enabling/disabling torque,
reading positions. It checks for stalls, and updating motor states.
It receives commands from the TaskHandler and executes them.
"""

from dataclasses import dataclass
from typing import Callable, Optional
from robot_arm.common.utils import *
from collections import defaultdict
import dynamixel_sdk as dxl


logger = setup_logging()


@dataclass
class MotorCommand:
    id: int
    action: str
    data: Optional[dict] = None
    callback: Optional[Callable] = None


class MotorController:
    def __init__(self):
        self.args = parse_args()
        self.packet_handler = None
        self.port_handler = None
        self.ids = DXL_IDS
        self.limits = parse_limits()
        self.home_positions = parse_home()
        self.port = self.args.port
        self._stall_counter = defaultdict(int)
        self._stall_threshold = 3
        self.moving_motors = {}
        self.stalled_motors = defaultdict(int)
        self.motor_positions = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0}
        if not self.limits:
            logger.warning(
                "No motor limits found. Please calibrate the robot arm first.")
            return

    def connect(self) -> bool:
        """
        Connect to the robot arm's motors and set up the communication.
        This function initializes the port handler and packet handler for
        communication with the Dynamixel motors.
        """

        # Initialize port handler and packet handler
        self.port_handler = dxl.PortHandler(self.port)
        self.packet_handler = dxl.PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            logger.error(f"Failed to open port: {self.port}")
            self.packet_handler = None
            self.port_handler = None
            return False

        if not self.port_handler.setBaudRate(BAUDRATE):
            logger.error(f"Failed to set baud rate: {BAUDRATE}")
            self.packet_handler = None
            self.port_handler = None
            return False

        self.enable_torque()
        return True

    def disconnect(self) -> None:
        """
        Disconnect from the robot arm's motors.
        This function closes the port handler to stop communication with the motors.
        """
        if self.port_handler:
            # Try to disable torque before closing the port
            try:
                self.disable_torque()
            except Exception as e:
                logger.warning(
                    f"Failed to disable torque during disconnect: {e}")

            self.port_handler.closePort()
            logger.info("Disconnected from the robot arm's motors.")
        else:
            logger.warning("Port handler is not initialized.")

    def enable_torque(self):
        """Enable torque for all motors."""

        for id in self.ids:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != dxl.COMM_SUCCESS:
                logger.error(
                    f"Failed to enable torque for motor {id}: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                logger.error(
                    f"Error enabling torque for motor {id}: {self.packet_handler.getRxPacketError(dxl_error)}")

    def disable_torque(self):
        """Disable torque for all motors."""

        for id in self.ids:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxl_comm_result != dxl.COMM_SUCCESS:
                logger.error(
                    f"Failed to disable torque for motor {id}: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                logger.error(
                    f"Error disabling torque for motor {id}: {self.packet_handler.getRxPacketError(dxl_error)}")

    def read_position(self, id) -> int:
        """
        Read the current position of a motor.
        :param id: The ID of the motor to read.
        :return: The current position of the motor.
        """
        position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, id, ADDR_PRESENT_POSITION)
        if dxl_comm_result != dxl.COMM_SUCCESS:
            logger.warning(
                f"Failed to read position from ID {id}: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            return None
        elif dxl_error != 0:
            logger.warning(
                f"Error reading position from ID {id}: {self.packet_handler.getRxPacketError(dxl_error)}")
            return None

        return position

    def move(self, id, position, speed=0) -> bool:
        """
        Move a motor to a specific position.
        :param id: The ID of the motor to move.
        :param position: The target position for the motor.
        :param speed: The speed at which to move the motor (optional).
        :return: True if the move was successful, False otherwise.
        """
        self.add_motor(id, position)

        # Check if position is within limits
        if id in self.limits:
            min_limit, max_limit = self.limits[id]
            if position < min_limit:
                logger.warning(
                    f"Position {position} below minimum limit {min_limit} for ID {id}. Using minimum limit.")
                position = min_limit
            elif position > max_limit:
                logger.warning(
                    f"Position {position} above maximum limit {max_limit} for ID {id}. Using maximum limit.")
                position = max_limit
        else:
            logger.warning(
                f"No limits found for ID {id}. Proceeding without limit check.")

        # Set speed if specified
        if speed > 0:
            dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
                self.port_handler, id, ADDR_PROFILE_VELOCITY, speed)

            if dxl_comm_result != dxl.COMM_SUCCESS:
                logger.warning(
                    f"Failed to set speed for ID {id}: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
                return False
            elif dxl_error != 0:
                logger.warning(
                    f"Error setting speed for ID {id}: {self.packet_handler.getRxPacketError(dxl_error)}")
                return False

        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, id, ADDR_GOAL_POSITION, position)

        if dxl_comm_result != dxl.COMM_SUCCESS:
            logger.warning(
                f"Failed to move motor {id} to position {position}: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            return False
        elif dxl_error != 0:
            logger.warning(
                f"Error moving motor {id} to position {position}: {self.packet_handler.getRxPacketError(dxl_error)}")
            return False

        logger.info(f"Motor {id} moved to position {position}.")
        self.moving_motors[id] = position
        return True

    def remove_motor(self, id):
        """
        Remove a motor from the moving motors dictionary.
        :param id: The ID of the motor to remove.
        """
        if id in self.moving_motors:
            del self.moving_motors[id]
            logger.debug(f"Motor {id} removed from moving motors.")
        else:
            logger.warning(f"Motor {id} not found in moving motors.")

    def reached_position(self, id, target_position) -> bool:
        """
        Check if a motor has reached its target position.
        :param id: The ID of the motor to check.
        :param target_position: The target position to check against.
        :return: True if the motor has reached the target position, False otherwise.
        """
        current_position = self.read_position(id)
        if current_position is None:
            return False
        return abs(current_position - target_position) < DXL_MOVING_STATUS_THRESHOLD

    def reset_stall_counter(self, id):
        """Reset the stall counter for a specific motor."""
        if id not in self._stall_counter:
            self._stall_counter[id] = 0
        else:
            self._stall_counter[id] = 0
        logger.debug(f"Stall counter for motor {id} reset.")

    def increment_stall_counter(self, id):
        """Increment the stall counter for a specific motor."""
        if id not in self._stall_counter:
            self._stall_counter[id] = 1
        else:
            self._stall_counter[id] += 1
        logger.debug(
            f"Stall counter for motor {id} incremented to {self._stall_counter[id]}.")

    def check_stall(self, id) -> bool:
        """
        Detects if the motor is stalled.
        :param id: The ID of the motor to check.
        :return: True if the motor is stalled, False otherwise.
        """
        logger.debug(f"Checking stall for motor {id}.")

        # Initialize stall counter if it doesn't exist
        if id not in self._stall_counter:
            self._stall_counter[id] = 0

        # Check velocity
        velocity = self._check_velocity(id)
        if velocity is None:  # Communication error
            return False

        if abs(velocity) > DXL_VELOCITY_STALL_THRESHOLD:
            self.reset_stall_counter(id)
            return False  # Still moving fine

        # Check current
        current = self._check_current(id)
        if current is None:  # Communication error
            return False

        if abs(current) > DXL_STALL_LOAD_THRESHOLD:
            self.increment_stall_counter(id)
        else:
            self.reset_stall_counter(id)

        if self._stall_counter[id] >= self._stall_threshold:
            logger.warning(
                f"Motor {id} forcibly stalled: velocity={velocity}, current={current * 2.69:.0f} mA")
            # Reset stall counter
            self.reset_stall_counter(id)
            return True

        return False

    def _check_current(self, id) -> int:
        """
        Check the current load of a motor.
        :param id: The ID of the motor to check.
        :return: The current load of the motor in mA, or None if communication fails.
        """
        current, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
            self.port_handler, id, ADDR_PRESENT_LOAD)
        logger.debug(f"Motor {id} current: {current * 2.69:.0f} mA")
        if dxl_comm_result != dxl.COMM_SUCCESS or dxl_error != 0:
            logger.warning(f"Failed to read current from motor {id}")
            return None
        if current > 32767:
            current -= 65536
        return current

    def _check_velocity(self, id) -> int:
        """
        Check the current velocity of a motor.
        :param id: The ID of the motor to check.
        :return: The current velocity of the motor, or None if communication fails.
        """
        velocity, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, id, ADDR_PRESENT_VELOCITY)
        if dxl_comm_result != dxl.COMM_SUCCESS or dxl_error != 0:
            logger.warning(f"Failed to read velocity from motor {id}")
            return None
        # Convert from 32-bit unsigned to signed integer
        if velocity > 0x7FFFFFFF:  # if highest bit is set (negative value)
            velocity = velocity - 0x100000000
        logger.debug(f"Motor {id} velocity: {velocity}")
        return velocity

    def add_motor(self, id, position):
        """
        Add a motor to the moving list.
        :param id: The ID of the motor to add.
        :param position: The target position for the motor.
        """
        if id not in self.moving_motors:
            self.moving_motors[id] = position
            logger.debug(
                f"Motor {id} added to moving list with target position {position}.")
        else:
            logger.warning(f"Motor {id} is already in the moving list.")

    def remove_motor(self, id):
        """
        Remove a motor from the moving list.
        :param id: The ID of the motor to remove.
        """
        if id in self.moving_motors:
            del self.moving_motors[id]
            logger.debug(f"Motor {id} removed from moving list.")
        else:
            logger.warning(f"Motor {id} is not in the moving list.")
