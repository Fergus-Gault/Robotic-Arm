"""
This module contains the task handler for the robot arm.
It manages a queue of tasks for the motors, processes them in a separate thread,
it schedules motor commands.
"""

from queue import Queue, Empty
import threading
from robot_arm.common.utils import *
from robot_arm.common.motors import MotorController, MotorCommand
import time
logger = setup_logging()


class TaskHandler:
    def __init__(self):
        self.motor_controller: MotorController = MotorController()
        self.task_queue = Queue()
        self.running = False
        self.last_update_time = time.time()
        self.time_between_updates = 0.1  # Time in seconds between updates

    def start_motor_task_thread(self):
        """
        Start the thread that processes motor tasks.
        This method initializes and starts a thread that will continuously
        process tasks from the task queue.
        """
        if not hasattr(self, 'task_thread') or not self.task_thread.is_alive():
            self.running = True
            self.task_thread = threading.Thread(
                target=self._task_thread_loop, daemon=True)
            self.task_thread.start()
            logger.info("Motor task thread started.")
        else:
            logger.warning("Motor task thread is already running.")

    def _task_thread_loop(self):
        """
        Main loop for processing tasks in the queue.
        This method runs in a separate thread and continuously processes tasks
        until the running flag is set to False.
        """
        while self.running:
            try:
                cmd: MotorCommand = self.task_queue.get(timeout=0.01)

                if cmd.action == "move":
                    self.motor_controller.move(
                        cmd.id, cmd.data['position'], cmd.data['speed'])
                elif cmd.action == "stop":
                    self.stop(cmd.id)
                elif cmd.action == "read_position":
                    logger.debug(f"Reading position for motor {cmd.id}.")
                    position = self.motor_controller.read_position(cmd.id)
                    if cmd.callback:
                        cmd.callback(cmd.id, position)
                elif cmd.action == "check_stall":
                    is_stalled = self.motor_controller.check_stall(cmd.id)
                    logger.debug(
                        f"Checking stall for motor {cmd.id}: {is_stalled}.")
                    if cmd.callback:
                        cmd.callback(cmd.id, is_stalled)

                self.task_queue.task_done()
                logger.debug("Processed task from the queue.")
            except Empty:
                pass

            if time.time() - self.last_update_time > self.time_between_updates:
                logger.debug(f"Tasks in queue: {self.task_queue.qsize()}")
                self.last_update_time = time.time()
                logger.debug("Updating motor positions and checking stalls.")
                self.update_motors()

    def connect(self):
        """
        Connect to the robot arm's motors and set up the communication.
        This function initializes the port handler and packet handler for
        communication with the Dynamixel motors.
        """
        self.running = True
        success = self.motor_controller.connect()
        self.start_motor_task_thread()
        return success

    def disconnect(self):
        """
        Disconnect from the robot arm's motors.
        This function closes the port handler to stop communication with the motors.
        """
        logger.debug("Disconnecting from motors...")
        self.task_queue.queue.clear()  # Clear the task queue before disconnecting
        self.running = False

        # Wait for the task thread to finish before disconnecting
        if hasattr(self, 'task_thread') and self.task_thread.is_alive():
            logger.debug("Waiting for task thread to finish...")
            self.task_thread.join(timeout=2.0)  # Wait up to 2 seconds
            if self.task_thread.is_alive():
                logger.warning("Task thread did not finish within timeout")

        self.motor_controller.disconnect()

    def add_move_task(self, motor_id, position, speed=0):
        """
        Add a move task to the queue.
        :param motor_id: The ID of the motor to move.
        :param position: The target position for the motor.
        """
        cmd = MotorCommand(action="move", id=motor_id,
                           data={'position': position, 'speed': speed})
        self.task_queue.put(cmd)
        logger.debug(
            f"Added move task for motor {motor_id} to position {position}.")

    def add_stop_task(self, motor_id):
        """
        Add a stop task to the queue.
        :param motor_id: The ID of the motor to stop.
        """
        cmd = MotorCommand(action="stop", id=motor_id)
        self.task_queue.put(cmd)
        logger.debug(f"Added stop task for motor {motor_id}.")

    def stop(self, id) -> None:
        """
        Stop a motor by setting its goal position to the current position.
        :param id: The ID of the motor to stop.
        """
        current_position = self.motor_controller.motor_positions[id]
        self.task_queue.queue.clear()  # Clear the task queue before stopping
        if current_position is not None:
            logger.info(f"Stopping motor {id} at position {current_position}.")
            self.add_move_task(id, current_position)
        else:
            logger.warning(
                f"Could not read position for motor {id}. Cannot stop.")

    def read_position(self, motor_id):
        """
        Read the current position of a motor.
        :param motor_id: The ID of the motor to read.
        :return: The current position of the motor.
        """
        cmd = MotorCommand(action="read_position", id=motor_id,
                           callback=self._update_position)
        self.task_queue.put(cmd)
        logger.debug(f"Added read position task for motor {motor_id}.")
        return self.motor_controller.motor_positions.get(motor_id, None)

    def enable_torque(self):
        """
        Enable torque for all motors.
        This function sends a command to enable torque for all motors connected to the robot arm.
        """
        self.motor_controller.enable_torque()
        logger.info("Torque enabled for all motors.")

    def disable_torque(self):
        """
        Disable torque for all motors.
        This function sends a command to disable torque for all motors connected to the robot arm.
        """
        self.motor_controller.disable_torque()
        logger.info("Torque disabled for all motors.")

    def return_to_home(self):
        """
        Return all motors to their home position.
        This function sends a command to return all motors to their home position.
        """
        logger.info("Returning all motors to home positions.")
        self.task_queue.queue.clear()  # Clear the task queue before returning to home
        for id, home_position in self.motor_controller.home_positions.items():
            if id in self.motor_controller.ids:
                self.add_move_task(id, home_position)
            else:
                logger.warning(f"Motor ID {id} not found in connected motors.")

    def reached_position(self, motor_id,  target_position):
        """
        Check if a motor has reached its target position.
        :param motor_id: The ID of the motor to check.
        :return: True if the motor has reached the target position, False otherwise.
        """
        current_position = self.motor_controller.motor_positions[motor_id]
        return abs(current_position - target_position) < DXL_MOVING_STATUS_THRESHOLD

    def check_stall(self, motor_id):
        """
        Add a task to check if a motor is stalled.
        :param motor_id: The ID of the motor to check.
        """
        cmd = MotorCommand(action="check_stall",
                           id=motor_id, callback=self._update_stall)
        self.task_queue.put(cmd)
        logger.debug(f"Added stall check task for motor {motor_id}.")

    def update_motors(self):
        to_remove = []
        for id, position in self.motor_controller.moving_motors.items():
            self.read_position(id)
            if self.reached_position(id, position):
                logger.info(f"Motor {id} reached position {position}.")
                to_remove.append(id)
            else:
                self.check_stall(id)
                if self.motor_controller.stalled_motors[id] == 1:
                    logger.warning(
                        f"Motor {id} is stalled at position {self.read_position(id)}.")
                    self.add_stop_task(id)
                    to_remove.append(id)

        for id in to_remove:
            self.motor_controller.remove_motor(id)

    def _update_stall(self, motor_id, is_stalled):
        """
        Update the stall status of a motor.
        :param motor_id: The ID of the motor to update.
        :param is_stalled: True if the motor is stalled, False otherwise.
        """
        if is_stalled:
            self.motor_controller.stalled_motors[motor_id] = 1
            logger.debug(f"Motor {motor_id} is stalled.")
        else:
            self.motor_controller.stalled_motors[motor_id] = 0

    def _update_position(self, motor_id, position):
        """
        Update the position of a motor.
        :param motor_id: The ID of the motor to update.
        :param position: The current position of the motor.
        """
        self.motor_controller.motor_positions[motor_id] = position

    def move_all_to_center(self):
        """
        Move all motors to their center position.
        This function sends a command to move all motors to their center position.
        """
        logger.info("Moving all motors to center positions.")
        self.task_queue.queue.clear()
        for id in self.motor_controller.ids:
            self.add_move_task(id, 2048)
