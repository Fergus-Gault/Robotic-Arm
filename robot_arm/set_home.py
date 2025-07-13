from robot_arm.common.utils import *
from robot_arm.common.arm import RobotArm


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
        for id in DXL_IDS:
            position = arm.read_position(id)
            # Initialize home position to current position
            home_positions[id] = position

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
