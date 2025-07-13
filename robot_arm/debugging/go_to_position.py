from robot_arm.common.arm import RobotArm
from robot_arm.common.utils import *
import time


def go_to_position():
    arm = RobotArm()
    success = arm.connect()
    logger = setup_logging()
    if not success:
        logger.error("Failed to connect to the robot arm.")
        return

    try:
        while True:
            try:
                user_input = input(
                    "Enter `motor_id position` (e.g., '1 1000'), 'r' to return to home, or 'q' to quit: ")

                if user_input.lower() == 'q':
                    logger.info("Exiting program")
                    break
                elif user_input.lower() == 'r':
                    logger.info("Returning to home positions")
                    arm.return_to_home()
                    continue

                parts = user_input.split()
                if len(parts) != 2:
                    logger.warning(
                        "Invalid input format. Please use: motor_id position")
                    continue

                motor_id = int(parts[0].strip())
                position = int(parts[1].strip())

                logger.info(f"Moving motor {motor_id} to position {position}")
                arm.move(motor_id, position)

                # Wait for the movement to complete
                time.sleep(1)

                current_position = arm.read_position(motor_id)
                logger.info(
                    f"Motor {motor_id} is now at position: {current_position}")

            except ValueError:
                logger.error(
                    "Invalid input. Motor ID should be an integer, position should be a number.")
            except Exception as e:
                logger.error(f"Error moving motor: {e}")

    except KeyboardInterrupt:
        logger.error(f"Program interrupted by user. Exiting...")

    finally:
        arm.return_to_home()
        arm.disconnect()


if __name__ == "__main__":
    go_to_position()
