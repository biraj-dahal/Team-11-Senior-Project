import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import numpy as np
import cv2
import time
import threading
from queue import Queue

def on_eos(bus, message, loop):
    """ End of stream callback """
    print('End of Stream')
    loop.quit()

def on_error(bus, message, loop):
    """ Error callback with detailed logging """
    err, debug_info = message.parse_error()
    print(f"Error: {err}, {debug_info}")
    loop.quit()

def on_new_sample(sink, user_data):
    """ Callback function to grab a frame from the pipeline and detect QR codes """
    frame_queue = user_data  # Unpack frame_queue from user_data
    sample = sink.emit("pull-sample")
    if sample:
        # Convert the sample to an OpenCV frame
        buffer = sample.get_buffer()
        caps = sample.get_caps()

        # Extract the caps structure (the first structure in the caps)
        structure = caps.get_structure(0)

        # Get width and height from caps and print them
        width = structure.get_value('width')
        height = structure.get_value('height')
        format = structure.get_value('format')
        print(f"Width: {width}, Height: {height}, Format: {format}")

        # Map the buffer to access the data
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if success:
            # Check if the buffer size matches the expected size
            frame = np.ndarray(
                (height, width, 3), dtype=np.uint8, buffer=map_info.data)

            # Log frame details to check if the frame is valid
            print(f"Frame shape: {frame.shape}, dtype: {frame.dtype}")

            # Detect QR codes
            qr_code_detector = cv2.QRCodeDetector()
            data, pts, qr_code = qr_code_detector.detectAndDecode(frame)

            if data:
                print(f"QR Code Data: {data}")
            else:
                print("Nothing: No QR code detected")

            # Put the frame into the queue to be processed by the main thread
            frame_queue.put(frame)

            # Unmap the buffer
            buffer.unmap(map_info)

    return Gst.FlowReturn.OK

def display_frames(frame_queue):
    """ Main thread function to display frames """
    while True:
        if not frame_queue.empty():
            frame = frame_queue.get()
            if frame is None:
                continue

            # Log frame details
            print(f"Displaying frame with shape: {frame.shape}")

            # Show the frame with the detected QR code
            cv2.imshow('RTSP Stream with QR Codes', frame)

            # Check for exit key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cv2.destroyAllWindows()

def start_rtsp_stream():
    # Initialize GStreamer
    Gst.init(None)

    # Create the pipeline for the RTSP stream with videoconvert (to handle different formats)
    pipeline = Gst.parse_launch(
        "rtspsrc location=rtsp://10.0.0.222/color ! decodebin ! videoconvert ! video/x-raw, format=RGB ! appsink name=sink emit-signals=True"
    )

    # Create a queue for passing frames between threads
    frame_queue = Queue()

    # Get the appsink element and set the callback function
    appsink = pipeline.get_by_name("sink")
    if appsink:
        appsink.connect("new-sample", on_new_sample, frame_queue)
    else:
        print("Failed to get appsink element")

    # Set up the loop and bus for handling events
    loop = GLib.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message::eos", on_eos, loop)
    bus.connect("message::error", on_error, loop)

    # Start the pipeline
    pipeline.set_state(Gst.State.PLAYING)

    # Start a separate thread for displaying frames
    display_thread = threading.Thread(target=display_frames, args=(frame_queue,))
    display_thread.start()

    try:
        loop.run()
    except:
        pass

    # Stop the pipeline when done
    pipeline.set_state(Gst.State.NULL)

if __name__ == "__main__":
    start_rtsp_stream()
