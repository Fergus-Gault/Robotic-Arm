from robot_arm.common.utils import *
from robot_arm.common.arm import RobotArm
import time
import keyboard


def calibrate():
    logger = setup_logging()

    arm = RobotArm()
    arm.connect()
    arm.disable_torque()

    logger.info(
        "Move the robot arm to the middle of its range, then press Enter.")
    time.sleep(1)
    if keyboard.is_pressed('enter'):
        pass

    # Dictionary to store min and max positions for each motor
    motor_limits = {}

    logger.info("Move each motor through its full range of motion.")
    logger.info(
        "The system will track the minimum and maximum positions automatically.")
    logger.info(
        "Press Enter when you've completed the full range for all motors.")

    # Initialize motor limits with the first read position
    for id in DXL_IDS:
        # Initialize min and max to current position
        motor_limits[id] = (2048, 2048)

    try:
        logger.info(
            "Moving motors to find min/max positions. Press Enter when done...")
        while True:
            all_positions = arm.task_handler.motor_controller.read_all_positions()

            for id in DXL_IDS:
                position = all_positions.get(id)
                if position is None or position <= 0:
                    # Fallback to individual read if bulk read failed
                    position = arm.read_position(id)
                    if position is None or position <= 0:
                        logger.error(f"Failed to read position for motor {id}")
                        continue

                logger.debug(f"Motor {id} position: {position}")
                current_min, current_max = motor_limits.get(
                    id, (position, position))
                motor_limits[id] = (
                    min(current_min, position), max(current_max, position))

            if keyboard.is_pressed('enter'):
                break

    except KeyboardInterrupt:
        pass

    # Display the captured limits
    logger.info("--- Recorded Motor Limits ---")
    for id, (min_pos, max_pos) in motor_limits.items():
        logger.info(f"Motor {id}: Min={min_pos}, Max={max_pos}")

    # Save all limits to file
    with open("robot_arm/configs/arm/motor_limits.txt", "w") as f:
        for id, (min_pos, max_pos) in motor_limits.items():
            f.write(f"{id},{min_pos},{max_pos}\n")

    logger.info(
        "Calibration complete! Motor limits saved to 'motor_limits.txt'")
    arm.disconnect()


if __name__ == "__main__":
    calibrate()
