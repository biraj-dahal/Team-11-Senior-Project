import sys
import os
import time
import logging
from robotic_arm.robot import Robot
import argparse

import robotic_arm.utilities as utilities
parser = argparse.ArgumentParser()
args = utilities.parseConnectionArguments(parser)


def setup_logging():
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler('robot_operation.log')
        ]
    )
    return logging.getLogger(__name__)


logger = setup_logging()
with utilities.DeviceConnection.createTcpConnection(args) as router:
        robot = Robot(router)
        
        
        logger.info("Robot initialization complete. Starting main operation loop.")
        
        while True:
            try:
        
                logger.info("Moving to home position...")
                if not robot.move_to_home_position():
                    logger.error("Failed to reach home position")
                    continue
                time.sleep(2)

                if not robot.open_gripper_with_speed():
                    logger.error("Failed to open gripper")
                    continue
                time.sleep(1)

        
                logger.info("Moving to scan position...")
                scan_position = robot.pre_defined_positions.get_position("QRSCAN")
                if not robot.move_to_angle_config(scan_position.angles):
                    logger.error("Failed to reach scan position")
                    continue
                time.sleep(2)


                logger.info("Scanning for package...")
                package_type = None
                while package_type is None:
                    package_type = robot.scan_qr_and_process()
                    if package_type is None:
                        logger.info("No QR code detected, retrying in 1 second...")
                        time.sleep(1)
                logger.info(f"Package type detected: {package_type}")


                if not robot.process_package(package_type):
                    logger.error("Failed to process package")
                    continue

                logger.info("Package delivery cycle complete. Starting next cycle...\n")
                time.sleep(2)  

            except Exception as e:
                logger.error(f"Error in operation cycle: {e}")
                time.sleep(5)  
                continue

            except KeyboardInterrupt:
                logger.info("Operation stopped by user")
            except Exception as e:
                logger.error(f"Critical error: {e}")
            finally:
                logger.info("Shutting down robot...")
                if 'robot' in locals():
                        robot.move_to_home_position()
                logger.info("Operation ended")

        


