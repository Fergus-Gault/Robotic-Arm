from robot_arm.common.utils import *
from robot_arm.common.arm import RobotArm
import time


def set_home():
    """
    Set and save the home position of the robot arm.
    This function allows the user to move motors to a desired home position
    and then save these positions to a file.
    """
    logger = setup_logging()
    arm = RobotArm()
    success = arm.connect()
    if not success:
        logger.error("Failed to connect to the robot arm.")
        return

    arm.disable_torque()
    logger.info("Robot arm connected and torque disabled.")
    logger.info(
        "Move the robot arm to the desired home position for each motor.")
    logger.info("Press Enter to record the position for each motor.")
    logger.info("Press Ctrl+C to exit at any time.")

    home_positions = {}
    try:
        # Initialize home positions for each motor
        while True:
            time.sleep(0.5)
            valid_positions = arm.task_handler.motor_controller.read_all_positions()

            # Filter out any zero positions (invalid readings)
            valid_positions = {id: pos for id, pos in valid_positions.items()
                               if pos != 0 and id in DXL_IDS}

            if len(valid_positions) == len(DXL_IDS):
                home_positions = valid_positions
                break

            logger.info("Some motor positions are 0. Retrying...")

    except KeyboardInterrupt:
        logger.info("Operation cancelled by user.")
        return

    finally:
        arm.disconnect()
        logger.info("Robot arm disconnected.")

    # Display the captured home positions
    logger.info("\n--- Recorded Home Positions ---")
    for id, position in home_positions.items():
        logger.info(f"Motor {id}: Position={position}")

    # Save home positions to a file
    with open("robot_arm/configs/arm/home_positions.txt", "w") as f:
        for id, position in home_positions.items():
            f.write(f"{id},{position}\n")

    logger.info("\nHome positions set! Values saved to 'home_positions.txt'")


if __name__ == "__main__":
    set_home()
