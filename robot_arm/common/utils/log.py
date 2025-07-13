import logging
from robot_arm.common.utils import *


def setup_logging():
    """
    Setup logging configuration for the robot arm utilities.
    This function configures the logging to output messages to the console
    with a specific format and log level.
    """
    args = parse_args()

    if args.debug:
        logging.basicConfig(
            level=logging.DEBUG,
            format='[%(levelname)s] %(message)s (%(filename)s:%(lineno)d)',
        )
    else:
        logging.basicConfig(
            level=logging.INFO,
            format='[%(levelname)s] %(message)s (%(filename)s:%(lineno)d)',
        )
    logger = logging.getLogger(__name__)
    return logger
