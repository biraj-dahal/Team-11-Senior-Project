import sys
import os
import time

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
        time.sleep(3)
        robot.open_gripper_with_speed()
        time.sleep(3)
        robot.move_to_angle_config(
            [345.845, 39.657, 192.208, 250.67, 345.695, 331.633, 97.461]
        )
        time.sleep(3)
        robot.close_gripper_with_speed(2)
        time.sleep(3)
        robot.move_to_home_position()


if __name__ == "__main__":
    main()
