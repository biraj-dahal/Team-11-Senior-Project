import asyncio
import json
from queue import Queue
from threading import Thread, Event
from robot import robot_script
import websockets
import time

datetime_format = "%Y-%m-%d %H:%M:%S"


message_queue = Queue()

robot_thread = Thread(target=robot_script, args=(message_queue,))


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
    while True:
        if not stop_flag.is_set():
            print("running automantic control")


def manual_control_target(stop_flag: Event):
    while True:
        if not stop_flag.is_set():
            print("running manual control")

def emergency_stop_target(stop_flag: Event):
    while True:
        if not stop_flag.is_set():
            print("running emergency stop")

# Start the communication thread
stop_flag = Event()
communcation_thread = Thread(target=communication_target, args=(message_queue, stop_flag))
communcation_thread.start()

# Run the automatic control thread by default
automatic_control_stop_flag = Event()
automatic_control_thread = Thread(target=automatic_control_target, args=(automatic_control_stop_flag,))
automatic_control_thread.start()

# Start the manual control thread
manual_control_stop_flag = Event()
manual_control_stop_flag.set()
manual_control_thread = Thread(target=manual_control_target, args=(manual_control_stop_flag,))
manual_control_thread.start()

# Start the emergency stop thread
emergency_stop_flag = Event()
emergency_stop_flag.set()
emergency_stop_thread = Thread(target=emergency_stop_target, args=(emergency_stop_flag,))
emergency_stop_thread.start()

while True:
    message = json.loads(message_queue.get())
    match message["control_type"]:
        case "manual_control":
        # TODO: stop the robot thread and start the emergency controls
            automatic_control_stop_flag.set()
            manual_control_stop_flag.clear()
            emergency_stop_flag.set()
        case "automatic_controls":
        # TODO: stop any existing manual_control or emergency_stop threads and restart automatic control thread
            automatic_control_stop_flag.clear()
            manual_control_stop_flag.set()
            emergency_stop_flag.set()
        case "emergency_stop":
        # TODO:: stop any existing manual_control and automatic control thread and initiate an emergency stop 
            automatic_control_stop_flag.set()
            manual_control_stop_flag.set()
            emergency_stop_flag.clear()   
        case _:
        # TODO: error out saying noto recognized type
         pass

    print(message)
        # time.sleep(100)


print(message)

stop_flag.set()


