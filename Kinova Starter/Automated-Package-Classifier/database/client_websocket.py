import json
from queue import Queue
from threading import Thread, Event
from robot import robot_script
from thread_functions import communication_target, automatic_control_target, manual_control_target, emergency_stop_target

datetime_format = "%Y-%m-%d %H:%M:%S"


message_queue = Queue()

robot_thread = Thread(target=robot_script, args=(message_queue,))


# If this is changed change thread_functions.py
uri = "ws://localhost:8000/robot_ws"


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
    data = message_queue.get()
    message = json.loads(data)
    control_type = message["control_type"]
    if control_type == "manual_control":
        # TODO: stop the robot thread and start the emergency controls
        automatic_control_stop_flag.set()
        manual_control_stop_flag.clear()
        emergency_stop_flag.set()
    elif control_type == "automatic_control":
        # TODO: stop any existing manual_control or emergency_stop threads and restart automatic control thread
        automatic_control_stop_flag.clear()
        manual_control_stop_flag.set()
        emergency_stop_flag.set()
    elif control_type == "emergency_stop":
        automatic_control_stop_flag.set()
        manual_control_stop_flag.set()
        emergency_stop_flag.clear()   

    print(message)


print(message)

stop_flag.set()


