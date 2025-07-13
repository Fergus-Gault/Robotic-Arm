from robot_arm.common.utils import *
import time
from robot_arm.common.arm import RobotArm


def move_range():
    arm = RobotArm()
    arm.connect()
    logger = setup_logging()

    try:

        arm.move_all_to_center()
        time.sleep(1)

        # Function to move a motor through its range
        def move_motor_through_range(id):
            logger.info(f"\nMoving motor ID {id} through positions...")
            motor_limits = arm.get_motor_limits(id)
            if motor_limits is None:
                logger.error(f"No limits found for motor ID {id}")
                return

            min_pos = motor_limits[0]
            max_pos = motor_limits[1]
            logger.info(f"Motor {id} limits: Min={min_pos}, Max={max_pos}")

            # Move to minimum position
            logger.info(f"Moving motor {id} to minimum position {min_pos}")
            arm.move(id, min_pos, arm.args.speed)
            while not arm.reached_position(id, min_pos):
                time.sleep(0.5)  # Increased delay to reduce queue flooding
            logger.info(f"Motor {id} reached minimum position")
            time.sleep(1)

            # Move to maximum position
            logger.info(f"Moving motor {id} to maximum position {max_pos}")
            arm.move(id, max_pos, arm.args.speed)
            while not arm.reached_position(id, max_pos):
                time.sleep(0.5)  # Increased delay to reduce queue flooding
            logger.info(f"Motor {id} reached maximum position")
            time.sleep(1)

            # Move to center position
            logger.info(f"Moving motor {id} to center position 2048")
            arm.move(id, 2048, arm.args.speed)
            while not arm.reached_position(id, 2048):
                time.sleep(0.5)  # Increased delay to reduce queue flooding
            logger.info(f"Motor {id} reached center position")
            time.sleep(1)

            logger.info(f"Completed range for motor ID {id}")

        # Move motor 1 through its range
        move_motor_through_range(1)

        # Move motor 4 to max position first (if needed for specific sequence)
        motor_4_limits = arm.get_motor_limits(4)
        if motor_4_limits:
            logger.info(
                f"Moving motor 4 to maximum position {motor_4_limits[1]}")
            arm.move(4, motor_4_limits[1], arm.args.speed)
            while not arm.reached_position(4, motor_4_limits[1]):
                time.sleep(0.5)

        # Move motor 3 through its range
        move_motor_through_range(3)

        # Move motor 4 through its range
        move_motor_through_range(4)

        # Move motor 5 through its range
        move_motor_through_range(5)

        # Move motor 6 through its range
        move_motor_through_range(6)

        # Return all motors to center position
        logger.info("\nReturning all motors to center position...")
        arm.move_all_to_center()

        motor_ids = arm.get_motor_ids()
        for id in motor_ids:
            while not arm.reached_position(id, 2048):
                time.sleep(0.1)
        logger.info("All motors returned to center position")

        # Move all motors to their home positions
        logger.info("\nMoving all motors to home positions...")
        arm.return_to_home()

        # Wait for all motors to reach home positions
        home_positions = arm.get_home_positions()
        for id in motor_ids:
            home_pos = home_positions.get(id)
            if home_pos is not None:
                while not arm.reached_position(id, home_pos):
                    time.sleep(0.5)
                logger.info(
                    f"Motor ID {id} has reached home position {home_pos}")
            else:
                logger.warning(f"No home position found for motor ID {id}")

        logger.info("Sequence completed!")

    except KeyboardInterrupt:
        logger.info(
            "\nKeyboard interrupt detected. Disabling torque and closing port...")
        arm.disconnect()
    finally:
        arm.disconnect()


if __name__ == "__main__":
    move_range()
