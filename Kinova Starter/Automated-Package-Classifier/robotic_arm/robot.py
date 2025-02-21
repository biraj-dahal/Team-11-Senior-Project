import sys
import threading
import time
import os
import re
import json

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2
from typing import List

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from vision.streamer_pipeline import RTSPStreamProcessor

class DefinedPositions:
    PERISHABLE = [292.318, 64.931, 178.571, 253.963, 36.948, 348.73, 344.96] # W
    HAZARDOUS = [165.682, 358.721, 174.79, 260.499, 64.21, 329.353, 43.232] # S
    RETURN = [131.384, 34.513, 126.049, 232.762, 53.654, 316.227, 60.587] # E
    MAINCONVEYER = [345.845, 39.657, 192.208, 250.67, 345.695, 331.633, 97.461] # ND
    QRSCAN = [342.552, 27.898, 196.617, 256.042, 350.27, 314.503, 93.662] #NU

class Robot:
    def __init__(self, router, proportional_gain=2.0, timeout_duration=20):

        self.proportional_gain = proportional_gain
        self.router = router
        self.timeout_duration = timeout_duration

        # Create base client using TCP router
        self.base = BaseClient(self.router)
        self.pre_defined_positions = DefinedPositions()

    def open_gripper_with_speed(self, gripper_speed=Base_pb2.GRIPPER_SPEED):
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Set speed to open gripper
        print("Opening gripper using speed command...")
        gripper_command.mode = gripper_speed
        finger.value = 0.1
        self.base.SendGripperCommand(gripper_command)
        gripper_request = Base_pb2.GripperRequest()

        # Wait for reported position to be opened
        gripper_request.mode = Base_pb2.GRIPPER_POSITION
        while True:
            gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
            if len(gripper_measure.finger):
                print(
                    "Current position is : {0}".format(gripper_measure.finger[0].value)
                )
                if gripper_measure.finger[0].value < 0.01:
                    break
            else:  # Else, no finger present in answer, end loop
                break

    # TODO: Add checking for success or failure of the command
    def close_gripper_with_speed(self, gripper_speed=Base_pb2.GRIPPER_SPEED):
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

    # Create closure to set an event after an END or an ABORT
    def _check_for_end_or_abort(self, e: threading.Event):
        """Return a closure checking for END or ABORT notifications

        Arguments:
        e -- event to signal when the action is completed
            (will be set when an END or ABORT occurs)
        """

        def check(notification, e=e):
            print("EVENT : " + Base_pb2.ActionEvent.Name(notification.action_event))
            if (
                notification.action_event == Base_pb2.ACTION_END
                or notification.action_event == Base_pb2.ACTION_ABORT
            ):
                e.set()

        return check

    def move_to_home_position(self):
        # Make sure the arm is in Single Level Servoing mode
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(base_servo_mode)

        # Move arm to ready position
        print("Moving the arm to a safe position")
        action_type = Base_pb2.RequestedActionType()
        action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
        action_list = self.base.ReadAllActions(action_type)
        action_handle = None

        # This might be wrong
        for action in action_list.action_list:
            if action.name == "Home":
                action_handle = action.handle

        if action_handle == None:
            print("Can't reach safe position. Exiting")
            sys.exit(0)

        # Notification Handle maybe move this into a separate function if needed by all commands
        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self._check_for_end_or_abort(e), Base_pb2.NotificationOptions()
        )

        self.base.ExecuteActionFromReference(action_handle)

        # Leave time to action to complete
        finished = e.wait(self.timeout_duration)
        self.base.Unsubscribe(notification_handle)

        if not finished:
            print("Timeout on action notification wait")
        return finished

    def move_to_angle_config(self, angle_config: List[float]) -> bool:
        print("Starting angular action movement ...")
        action = Base_pb2.Action()
        action.name = "Example angular action movement"
        action.application_data = ""

        actuator_count = self.base.GetActuatorCount()

        if len(angle_config) != actuator_count.count:
            # Replace with error
            print(
                "Number of joint angles provided and the actuator count is different!"
            )
            return False

        # Place arm straight up
        for joint_id in range(actuator_count.count):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = joint_id
            joint_angle.value = angle_config[joint_id]

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self._check_for_end_or_abort(e), Base_pb2.NotificationOptions()
        )

        print("Executing action")
        self.base.ExecuteAction(action)

        print("Waiting for movement to finish ...")
        finished = e.wait(self.timeout_duration)
        self.base.Unsubscribe(notification_handle)

        if finished:
            print("Angular movement completed")
        else:
            print("Timeout on action notification wait")
        return finished
    

    
    def scan_qr_and_process(self):
        rtsp_url = "rtsp://10.0.0.222/color" 
        stream_processor = RTSPStreamProcessor(rtsp_url)
        stream_data = ""
        try:
            stream_data_str = stream_processor.start_rtsp_stream()
            stream_data = self.__convert_to_dict(stream_data_str)
        except:
            print("Error retriving data from the RTSPStreamProcessor")
        
        return stream_data['package_type']

    def __convert_to_dict(self, input_str):

        corrected_str = re.sub(r'([a-zA-Z_][a-zA-Z0-9_]*)\s*:', r'"\1":', input_str) 
        corrected_str = re.sub(r':\s*([a-zA-Z0-9_]+)(?=\s*[,}])', r':"\1"', corrected_str) 
        python_dict = ""
        try:
            python_dict = json.loads(corrected_str)
            return python_dict
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON: {e}")
            return None
        
        return python_dict

    
