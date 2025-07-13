"""
This module contains the Arm class, with the responsibility of managing high level operations
for the robot arm, including connnecting to the motors, and queuing tasks for the motors.
There is no direct interation with the motors in this class. It adds tasks to the TaskHandler,
and the TaskHandler processes these tasks in a separate thread.
"""

from robot_arm.common.task_handler import TaskHandler
from robot_arm.common.utils import parse_args


class RobotArm:
    def __init__(self):
        self.task_handler: TaskHandler = TaskHandler()
        self.args = parse_args()

    def connect(self):
        """
        Connect to the robot arm's motors and set up the communication.
        This function initializes the port handler and packet handler for
        communication with the Dynamixel motors.
        """
        return self.task_handler.connect()

    def disconnect(self):
        """
        Disconnect from the robot arm's motors.
        This function closes the port handler to stop communication with the motors.
        """
        self.task_handler.disconnect()

    def move(self, motor_id, position, speed=0):
        """
        Move a motor to a specified position.
        :param motor_id: The ID of the motor to move.
        :param position: The target position for the motor.
        """
        self.task_handler.add_move_task(motor_id, position, speed)

    def stop(self, motor_id):
        """
        Stop a motor.
        :param motor_id: The ID of the motor to stop.
        """
        self.task_handler.add_stop_task(motor_id)

    def read_position(self, motor_id):
        """
        Read the current position of a motor.
        :param motor_id: The ID of the motor to read.
        :return: The current position of the motor.
        """
        return self.task_handler.read_position(motor_id)

    def reached_position(self, motor_id, position):
        """
        Check if a motor has reached a specified position.
        :param motor_id: The ID of the motor to check.
        :param position: The target position to check against.
        :return: True if the motor is at the target position, False otherwise.
        """
        return self.task_handler.reached_position(motor_id, position)

    def enable_torque(self):
        """
        Enable torque for all motors.
        This function sends a command to enable torque for all motors connected to the robot arm.
        """
        self.task_handler.enable_torque()

    def disable_torque(self):
        """
        Disable torque for all motors.
        This function sends a command to disable torque for all motors connected to the robot arm.
        """
        self.task_handler.disable_torque()

    def return_to_home(self):
        """
        Return all motors to their home position.
        This function sends a command to return all motors to their home position.
        """
        self.task_handler.return_to_home()

    def move_all_to_center(self):
        """
        Move all motors to their center position.
        This function sends a command to move all motors to their center position.
        """
        self.task_handler.move_all_to_center()

    def get_motor_limits(self, motor_id=None):
        """
        Get the limits for a specific motor or all motors.
        :param motor_id: The ID of the motor to get limits for. If None, returns all limits.
        :return: The limits for the specified motor or all motors.
        """
        if motor_id is None:
            return self.task_handler.motor_controller.limits
        return self.task_handler.motor_controller.limits.get(motor_id, None)

    def get_home_positions(self, motor_id=None):
        """
        Get the home position for a specific motor or all motors.
        :param motor_id: The ID of the motor to get home position for. If None, returns all home positions.
        :return: The home position for the specified motor or all motors.
        """
        if motor_id is None:
            return self.task_handler.motor_controller.home_positions
        return self.task_handler.motor_controller.home_positions.get(motor_id, None)

    def get_motor_ids(self):
        """
        Get the list of motor IDs.
        :return: List of motor IDs.
        """
        return self.task_handler.motor_controller.ids
