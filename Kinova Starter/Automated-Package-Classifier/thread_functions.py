from queue import Queue
from threading import Event
import asyncio
import json
import websockets
from read_pickup_drop import automatic_control_function
from robotic_arm.robot import Robot
from robotic_arm import utilities
import argparse

parser = argparse.ArgumentParser()
args = utilities.parseConnectionArguments(parser)
# if this is changed change client_websocket.py
uri = "ws://localhost:8000/robot_ws"

# Main thread/ Web Socket server
async def communication_function(out_message_queue: Queue, stop_flag: Event):
    first_contact = True
    while not stop_flag.is_set():
        async with websockets.connect(uri) as ws:
            if first_contact:
                await ws.send(json.dumps({
                    "type": "identification",
                    "identity": "robot"
                }))
                # will be identity saved. We can ignore this
                message = await ws.recv()
                print(message)
                first_contact = False
                continue
            message = await ws.recv()
            out_message_queue.put(message)

def communication_target(out_message_queue: Queue, stop_flag):
    asyncio.run(communication_function(out_message_queue, stop_flag))

def automatic_control_target(stop_flag: Event):
    while not stop_flag.is_set():
        automatic_control_function(stop_flag)

def manual_control_target(stop_flag: Event, message_queue: Queue):
    print("manual_thread")
    while not stop_flag.is_set():
        if stop_flag.is_set():
            break
        servo_config = message_queue.get()
        with utilities.DeviceConnection.createTcpConnection(args) as router:
            robot = Robot(router)
            if stop_flag.is_set():
                break
            robot.move_to_angle_config(servo_config)
            # print("running manual control")

def emergency_stop_target(stop_flag: Event):
    while stop_flag.is_set():
        with utilities.DeviceConnection.createTcpConnection(args) as router:
            robot = Robot(router)
            if stop_flag.is_set():
                break
            robot.open_gripper_with_speed()
