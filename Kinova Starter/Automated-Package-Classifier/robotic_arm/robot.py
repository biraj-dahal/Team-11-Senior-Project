import sys
import threading
import time
import os
import re
import json
import logging, logging.config
import yaml
from pathlib import Path
from typing import List, Optional, Dict, Any
from dataclasses import dataclass

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2

from .vision_controller import VisionController

BASE_DIR = Path(__file__).resolve().parent.parent
CONFIG_PATH = BASE_DIR / "config.yaml"
LOGGING_CONFIG_PATH = BASE_DIR / "logging.yaml"
if LOGGING_CONFIG_PATH.exists():
    with open(LOGGING_CONFIG_PATH, "r") as f:
        logging.config.dictConfig(yaml.safe_load(f))



def load_config():
    if CONFIG_PATH.exists():
        with open(CONFIG_PATH, "r") as f:
            return yaml.safe_load(f)
    else:
        raise FileNotFoundError(f"Missing configuration file: {CONFIG_PATH}")

config = load_config()

@dataclass
class RobotPosition:
    angles: List[float]
    name: str
    description: str

class DefinedPositions:
    def __init__(self):
        self.positions = {
            'PERISHABLE': RobotPosition(
                [233.076, 62.818, 208.56, 300.992, 328.957, 294.747, 348.297], # box 1
                'PERISHABLE',
                'Position for perishable items (W)'
            ),
            'HAZARDOUS': RobotPosition(
                [273.054, 75.953, 207.013, 311.781, 326.391, 295.177, 348.8], # box2
                'HAZARDOUS',
                'Position for hazardous items (S)'
            ),
            'RETURN': RobotPosition(
                [220.444, 81.122, 209.759, 347.953, 326.536, 273.377, 278.236], # box3
                'RETURN',
                'Position for return items (E)'
            ),
            'MAINCONVEYER': RobotPosition(
                [343.095, 59.301, 212.683, 284.752, 323.777, 309.359, 25.482],
                'MAINCONVEYER',
                'Main conveyer position (ND)'
            ),
            'QRSCAN': RobotPosition(
                [342.552, 27.898, 196.617, 256.042, 350.27, 314.503, 93.662],
                'QRSCAN',
                'QR code scanning position (NU)'
            ),
            'AUTOMATEDHOME': RobotPosition(
                [0.411, 0, 0.535, 180, 0, 90],
                'AUTOMATEDHOME',
                'YOLO scanning position'
            ),
            'HANDLE': RobotPosition(
                [341.645, 54.046, 214.396, 285.337, 326.087, 303.654, 24.285],
                'HANDLE',
                'Handle position (NU)'
            ),
            'SCAN_HANDLE': RobotPosition(
                [348.526, 33.124, 199.572, 331.918, 348.663, 244.222, 89.568],
                'SCAN_HANDLE',
                'Scan handle position (NU)'
            ),
            'PERIS_HANDLE': RobotPosition(
                [7.593, 35.262, 189.871, 243.091, 351.545, 335.521,106.426],
                'PERIS_HANDLE',
                'Perishable handle position (NU)'
            ),
            'SCAN_BOX': RobotPosition(
                [239.149, 54.587, 210.973, 338.643, 331.151, 252.81, 336.53],
                'SCAN_BOX',
                'SCAN_BOX position'
            )
        }

    def get_position(self, position_name: str) -> RobotPosition:
        if position_name not in self.positions:
            raise ValueError(f"Unknown position: {position_name}")
        return self.positions[position_name]

class Robot:
    def __init__(self, router, rtsp_url: str = config.get("rtsp_url", "rtsp://10.0.0.222/color"), 
                 proportional_gain: float = config.get("proportional_gain", 2.0), timeout_duration: int = config.get("timeout_duration", 20)):
 
        self.logger = logging.getLogger(__name__)
        self.proportional_gain = proportional_gain
        self.router = router
        self.timeout_duration = timeout_duration
        
        # Initialize base client for robot control
        self.base = BaseClient(self.router)
        
        #Initialize base cycle client for cartesian correspondence
        self.base_cycle = BaseCyclicClient(router)
        self.pre_defined_positions = DefinedPositions()
        
        # Initialize vision system
        self.vision_controller = VisionController(rtsp_url)
        
        # Threading protection
        self._movement_lock = threading.Lock()
        
        self.logger.info("Robot controller initialized")

    def open_gripper_with_speed(self, gripper_speed=Base_pb2.GRIPPER_SPEED) -> bool:

        try:
            with self._movement_lock:
                gripper_command = Base_pb2.GripperCommand()
                finger = gripper_command.gripper.finger.add()
                
                self.logger.info("Opening gripper...")
                gripper_command.mode = gripper_speed
                finger.value = 0.1
                self.base.SendGripperCommand(gripper_command)
                
                # Wait for completion
                gripper_request = Base_pb2.GripperRequest()
                gripper_request.mode = Base_pb2.GRIPPER_POSITION
                
                start_time = time.time()
                while time.time() - start_time < self.timeout_duration:
                    gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
                    if len(gripper_measure.finger):
                        current_pos = gripper_measure.finger[0].value
                        self.logger.debug(f"Current gripper position: {current_pos}")
                        if current_pos < 0.01:
                            return True
                    else:
                        break
                    time.sleep(0.1)
                
                self.logger.warning("Gripper operation timed out")
                return False
                
        except Exception as e:
            self.logger.error(f"Error opening gripper: {e}")
            return False

    def close_gripper_with_speed(self, gripper_speed=Base_pb2.GRIPPER_SPEED) -> bool:

        # Create the GripperCommand we will send
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Set speed to close gripper
        print("Closing gripper using speed command...")
        gripper_command.mode = gripper_speed
        finger.value = -0.1
        self.base.SendGripperCommand(gripper_command)
        gripper_request = Base_pb2.GripperRequest()

        # Wait for reported speed to be 0
        gripper_request.mode = Base_pb2.GRIPPER_POSITION
        gripper_request.mode = gripper_speed
        while True:
            gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
            if len(gripper_measure.finger):
                print("Current speed is : {0}".format(gripper_measure.finger[0].value))
                if gripper_measure.finger[0].value == 0.0:
                    break
            else:  # Else, no finger present in answer, end loop
                break

    def _check_for_end_or_abort(self, e: threading.Event):

        def check(notification, e=e):
            event_name = Base_pb2.ActionEvent.Name(notification.action_event)
            self.logger.debug(f"Robot event: {event_name}")
            if (notification.action_event == Base_pb2.ACTION_END or 
                notification.action_event == Base_pb2.ACTION_ABORT):
                e.set()
        return check

    def get_cartesian_pose(self) -> List[float]:
        self.logger.info("Fetching Cartesian position")
        action = Base_pb2.Action()
        action.name = "Example Cartesian fetch"
        action.application_data = "robits fetch"

        feedback = self.base_cycle.RefreshFeedback()

        return [feedback.base.tool_pose_x,
        feedback.base.tool_pose_y,
        feedback.base.tool_pose_z,
        feedback.base.tool_pose_theta_x,
        feedback.base.tool_pose_theta_y,
        feedback.base.tool_pose_theta_z
        ]

    def go_to_cartesian(self, coordinates: List[float]) -> bool:

        try:
            with self._movement_lock:
                x, y, z, theta_x, theta_y, theta_z = coordinates
                self.logger.info("Starting cartesian movement...")
                action = Base_pb2.Action()
                action.name = "RPC Programmed Cartesian Action"
                action.application_data = ""

                feedback = self.base_cycle.RefreshFeedback()

                cartesian_pose = action.reach_pose.target_pose
                cartesian_pose.x = x       # (meters)
                cartesian_pose.y = y
                cartesian_pose.z = z
                cartesian_pose.theta_x = theta_x  # (degrees)
                cartesian_pose.theta_y = theta_y
                cartesian_pose.theta_z = theta_z 

                # Execute movement
                e = threading.Event()
                notification_handle = self.base.OnNotificationActionTopic(
                    self._check_for_end_or_abort(e), 
                    Base_pb2.NotificationOptions()
                )

                self.base.ExecuteAction(action)
                finished = e.wait(self.timeout_duration)
                self.base.Unsubscribe(notification_handle)

                if not finished:
                    self.logger.warning("Cartesian movement timed out")
                return finished

        except Exception as e:
            self.logger.error(f"Error in angular movement: {e}")
            return False

    def go_to_automated_home(self) -> bool:
        pos = self.pre_defined_positions.get_position("AUTOMATEDHOME").angles
        self.go_to_cartesian(pos)

    def move_to_home_position(self) -> bool:

        try:
            with self._movement_lock:
                # Set servoing mode
                base_servo_mode = Base_pb2.ServoingModeInformation()
                base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
                self.base.SetServoingMode(base_servo_mode)

                self.logger.info("Moving to home position...")
                action_type = Base_pb2.RequestedActionType()
                action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
                action_list = self.base.ReadAllActions(action_type)
                
                # Find home position
                action_handle = None
                for action in action_list.action_list:
                    if action.name == "Home":
                        action_handle = action.handle
                        break

                if action_handle is None:
                    self.logger.error("Home position not found")
                    return False

                # Execute movement
                e = threading.Event()
                notification_handle = self.base.OnNotificationActionTopic(
                    self._check_for_end_or_abort(e), 
                    Base_pb2.NotificationOptions()
                )

                self.base.ExecuteActionFromReference(action_handle)
                finished = e.wait(self.timeout_duration)
                self.base.Unsubscribe(notification_handle)

                if not finished:
                    self.logger.warning("Home position movement timed out")
                return finished

        except Exception as e:
            self.logger.error(f"Error moving to home position: {e}")
            return False

    def move_to_angle_config(self, angle_config: List[float]) -> bool:

        try:
            with self._movement_lock:
                self.logger.info("Starting angular movement...")
                action = Base_pb2.Action()
                action.name = "Angular movement"
                action.application_data = ""

                actuator_count = self.base.GetActuatorCount()

                if len(angle_config) != actuator_count.count:
                    self.logger.error("Incorrect number of joint angles provided")
                    return False

                # Set joint angles
                for joint_id in range(actuator_count.count):
                    joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
                    joint_angle.joint_identifier = joint_id
                    joint_angle.value = angle_config[joint_id]

                # Execute movement
                e = threading.Event()
                notification_handle = self.base.OnNotificationActionTopic(
                    self._check_for_end_or_abort(e), 
                    Base_pb2.NotificationOptions()
                )

                self.base.ExecuteAction(action)
                finished = e.wait(self.timeout_duration)
                self.base.Unsubscribe(notification_handle)

                if not finished:
                    self.logger.warning("Angular movement timed out")
                return finished

        except Exception as e:
            self.logger.error(f"Error in angular movement: {e}")
            return False

    def scan_qr_and_process(self) -> Optional[str]:
        try:
            self.logger.info("Starting QR code scan")
            qr_data = self.vision_controller.scan_qr()
            
            if qr_data is None:
                self.logger.info("No QR code detected")
                return None
                
            if 'package_type' not in qr_data:
                self.logger.info("Invalid QR data format - missing package_type")
                return None
                
            return qr_data['package_type']
            
        except Exception as e:
            self.logger.error(f"Error scanning QR code: {e}")
            return None

    def process_package(self, package_type: str) -> bool:
        try:

            if package_type == "perishable":
                target_pos = self.pre_defined_positions.get_position("PERISHABLE")
            elif package_type == "hazardous":
                target_pos = self.pre_defined_positions.get_position("HAZARDOUS")
            else:
                target_pos = self.pre_defined_positions.get_position("RETURN")

            if not self.move_to_angle_config(self.pre_defined_positions.get_position("SCAN_HANDLE").angles):
                self.logger.error("Failed to move to Scan Handle position")
                return False
            time.sleep(2)
            
            if package_type == "perishable":
                if not self.move_to_angle_config(self.pre_defined_positions.get_position("PERIS_HANDLE").angles):
                    self.logger.error("Failed to move to peris position")
                    return False
                time.sleep(2)

            else:
                # Move to the main conveyor and close the gripper
                if not self.move_to_angle_config(self.pre_defined_positions.get_position("HANDLE").angles):
                    self.logger.error("Failed to move to handle position")
                    return False
                time.sleep(2)
                    

                if not self.move_to_angle_config(self.pre_defined_positions.get_position("MAINCONVEYER").angles):
                    self.logger.error("Failed to move to main conveyor position")
                    return False
                time.sleep(2)

            self.close_gripper_with_speed()
        
            time.sleep(2)

            if not self.move_to_home_position():
                self.logger.error("Failed to return to home position")
                return False
            time.sleep(2)
            
            if not self.move_to_angle_config(self.pre_defined_positions.get_position("SCAN_BOX").angles):
                    self.logger.error("Failed to scan box")
                    return False
            time.sleep(2)

            if not self.move_to_angle_config(target_pos.angles):
                self.logger.error("Failed to move to target position")
                return False
            time.sleep(2)

            if not self.open_gripper_with_speed():
                self.logger.error("Failed to open gripper")
                return False
            time.sleep(2)

            if not self.move_to_home_position():
                self.logger.error("Failed to return to home position")
                return False
            time.sleep(2)

            return True

        except Exception as e:
            self.logger.error(f"Error processing package: {e}")
            return False