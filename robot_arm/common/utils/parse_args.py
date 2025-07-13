import argparse


def parse_args():
    parser = argparse.ArgumentParser(
        description="Parse command line arguments for the script.")

    parser.add_argument('--port', type=str, default='COM1',
                        help='Serial port to connect to the robot.')
    parser.add_argument('--speed', type=int, default=100,
                        help='Speed for the robot arm movements.')
    parser.add_argument('--debug', action='store_true',
                        help='Enable debug mode for detailed logging.')

    args = parser.parse_args()
    return args
