import sys
import os
import time

from .robotic_arm.robot import Robot
import argparse

import robotic_arm.utilities as utilities
parser = argparse.ArgumentParser()
args = utilities.parseConnectionArguments(parser)

with utilities.DeviceConnection.createTcpConnection(args) as router:
        robot = Robot(router)
        while True:
                robot.move_to_home_position()
                time.sleep(3)
                robot.open_gripper_with_speed()
                time.sleep(3)
                robot.move_to_angle_config(
                robot.pre_defined_positions.QRSCAN
                )
                position = robot.scan_qr_and_process()

                time.sleep(3)

                robot.move_to_angle_config(robot.pre_defined_positions.MAINCONVEYER)
                time.sleep(3)
                robot.close_gripper_with_speed(2)
                time.sleep(3)
                robot.move_to_home_position()
                time.sleep(3)

                if position == 'perishable':
                        robot.move_to_angle_config(robot.pre_defined_positions.PERISHABLE)
                elif position == 'hazardous':
                        robot.move_to_angle_config(robot.pre_defined_positions.HAZARDOUS)
                else:
                        robot.move_to_angle_config(robot.pre_defined_positions.RETURN)
                time.sleep(3)
                robot.open_gripper_with_speed()
                time.sleep(3)

        
