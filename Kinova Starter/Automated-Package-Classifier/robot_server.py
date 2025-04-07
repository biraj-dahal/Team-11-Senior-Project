import sys
import os
import time
import logging
from robotic_arm.robot import Robot
import argparse
from threading import Event

import robotic_arm.utilities as utilities
parser = argparse.ArgumentParser()
args = utilities.parseConnectionArguments(parser)

import logging.config
import yaml

def setup_logging():
    with open('logging.yaml', 'r') as f:
        config = yaml.safe_load(f)
    logging.config.dictConfig(config)
    return logging.getLogger(__name__)

logger = setup_logging()

def automatic_control_function(stop_flag: Event):
    with utilities.DeviceConnection.createTcpConnection(args) as router:
            robot = Robot(router)
            
            
            logger.info("Robot initialization complete. Starting main operation loop.")
            
            try:
        
                logger.info("Moving to home position...")
                
                if stop_flag.is_set():
                    return
        
                if not robot.move_to_home_position():
                    logger.error("Failed to reach home position")
                    return
                time.sleep(2)

                if stop_flag.is_set():
                    return

                if not robot.open_gripper_with_speed():
                    logger.error("Failed to open gripper")
                    return
                time.sleep(1)

                if stop_flag.is_set():
                    return
 
                logger.info("Moving to scan position...")
                scan_position = robot.pre_defined_positions.get_position("QRSCAN")
                if stop_flag.is_set():
                    return
                if not robot.move_to_angle_config(scan_position.angles):
                    logger.error("Failed to reach scan position")
                    return
                time.sleep(2)


                logger.info("Scanning for package...")
                package_type = None
                while package_type is None:
                    if stop_flag.is_set():
                        return

                    package_type = robot.scan_qr_and_process()
                    if package_type is None:
                        logger.info("No QR code detected, retrying in 1 second...")
                        time.sleep(1)
                logger.info(f"Package type detected: {package_type}")

                if stop_flag.is_set():
                    return
                if not robot.process_package(package_type):
                    logger.error("Failed to process package")
                    return

                logger.info("Package delivery cycle complete. Starting next cycle...\n")
                time.sleep(2)  

            except Exception as e:
                logger.error(f"Error in operation cycle: {e}")
                time.sleep(5)  
            except KeyboardInterrupt:
                logger.info("Operation stopped by user")
            except Exception as e:
                logger.error(f"Critical error: {e}")
            finally:
                logger.info("Shutting down robot...")
                if 'robot' in locals():
                        robot.move_to_home_position()
                logger.info("Operation ended")

            


