import sys
import threading
import time

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2
from typing import List


class Robot:
    def __init__(self, router, proportional_gain=2.0, timeout_duration=20):

        self.proportional_gain = proportional_gain
        self.router = router
        self.timeout_duration = timeout_duration

        # Create base client using TCP router
        self.base = BaseClient(self.router)

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
