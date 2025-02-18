import sys
import os

from robot import Robot

# Maximum allowed waiting time during actions (in seconds)


def main():
    # Import the utilities helper module
    import argparse

    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    parser = argparse.ArgumentParser()
    args = utilities.parseConnectionArguments(parser)

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        robot = Robot(router)
        robot.move_to_home_position()
        robot.move_to_angle_config([0, 0, 0, 0, 0, 0])
        robot.close_gripper_with_speed()
        robot.move_to_home_position()


if __name__ == "__main__":
    main()
