import json
from queue import Queue
from threading import Thread, Event
from thread_functions import communication_target, automatic_control_target, manual_control_target, emergency_stop_target

datetime_format = "%Y-%m-%d %H:%M:%S"

message_queue = Queue()

# If this is changed change thread_functions.py
uri = "ws://localhost:8000/robot_ws"


# Start the communication thread
stop_flag = Event()
communcation_thread = Thread(target=communication_target, args=(message_queue, stop_flag))
communcation_thread.start()

# Run the automatic control thread by default
automatic_control_stop_flag = Event()
automatic_control_stop_flag.set()
automatic_control_thread = Thread(target=automatic_control_target, args=(automatic_control_stop_flag,))

# Start the manual control thread
manual_control_stop_flag = Event()
manual_control_message_queue = Queue()
manual_control_stop_flag.set()
manual_control_thread = Thread(target=manual_control_target, args=(manual_control_stop_flag, manual_control_message_queue))

# Start the emergency stop thread
emergency_stop_flag = Event()
emergency_stop_flag.set()
emergency_stop_thread = Thread(target=emergency_stop_target, args=(emergency_stop_flag,))

while True:
    print("waiting for message")
    data = message_queue.get()
    message = json.loads(data)
    print("Message recieved", message)
    control_type = message["control_type"]
    if control_type == "manual_control":
        # TODO: stop the robot thread and start the emergency controls
        # Stop send flag to threads stop
        automatic_control_stop_flag.set()
        emergency_stop_flag.set()

        # Join the threads to main thread
        if automatic_control_thread.is_alive():
            automatic_control_thread.join()
        if emergency_stop_thread.is_alive():
            emergency_stop_thread.join()
        print("joined to manual control")


        # Start the manual control thread
        manual_control_stop_flag.clear()
        manual_control_thread = Thread(target=manual_control_target, args=(manual_control_stop_flag, manual_control_message_queue))
        manual_control_thread.start()

        # Send the input
        print("sending message to manual thread", message)
        manual_control_message_queue.put(message)
    elif control_type == "automatic_control":
        # TODO: stop any existing manual_control or emergency_stop threads and restart automatic control thread
        manual_control_stop_flag.set()
        emergency_stop_flag.set()
        manual_control_message_queue.put("")

        # Join the threads to main thread
        if manual_control_thread.is_alive():
            manual_control_thread.join()
        print("joined")
        if emergency_stop_thread.is_alive():
            emergency_stop_thread.join()
        print("joined to automatic control")


        # Start the automatic control thread
        automatic_control_stop_flag.clear()
        automatic_control_thread = Thread(target=automatic_control_target, args=(automatic_control_stop_flag,))
        automatic_control_thread.start()

    elif control_type == "emergency_stop":
        automatic_control_stop_flag.set()
        manual_control_stop_flag.set()

        # Join the threads to main thread
        manual_control_thread.join()
        automatic_control_thread.join()



        emergency_stop_flag.clear()   
        emergency_stop_thread = Thread(target=emergency_stop_target, args=(emergency_stop_flag,))
        emergency_stop_thread.start()

    print(message)


print(message)

stop_flag.set()


