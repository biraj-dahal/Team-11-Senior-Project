import gi
import numpy as np
import cv2
import threading
from queue import Queue
from gi.repository import Gst, GLib
import json
import re

gi.require_version('Gst', '1.0')


class RTSPStreamProcessor:
    def __init__(self, rtsp_url):
        """ Initialize the RTSP stream processor with the RTSP URL """
        self.rtsp_url = rtsp_url
        self.frame_queue = Queue()
        self.loop = GLib.MainLoop()
        self.pipeline = None
        self.display_thread = None
        self.bus = ""
        self.data = ""

    def on_eos(self, bus, message):
        """ End of stream callback """
        print('End of Stream')
        self.loop.quit()

    def on_error(self, bus, message):
        """ Error callback with detailed logging """
        err, debug_info = message.parse_error()
        print(f"Error: {err}, {debug_info}")
        self.loop.quit()

    def on_new_sample(self, sink, user_data):
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
                    self.data = data
                    self.on_eos(bus=self.bus, message="Data detected")
                else:
                    print("Nothing: No QR code detected")

                # Put the frame into the queue to be processed by the main thread
                frame_queue.put(frame)

                # Unmap the buffer
                buffer.unmap(map_info)

        return Gst.FlowReturn.OK

    def display_frames(self):
        """ Main thread function to display frames """
        while True:
            if not self.frame_queue.empty():
                frame = self.frame_queue.get()
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

    def start_rtsp_stream(self):
        """ Start the RTSP stream pipeline """
        # Initialize GStreamer
        Gst.init(None)

        # Create the pipeline for the RTSP stream with videoconvert (to handle different formats)
        self.pipeline = Gst.parse_launch(
            f"rtspsrc location={self.rtsp_url} ! decodebin ! videoconvert ! video/x-raw, format=RGB ! appsink name=sink emit-signals=True"
        )

        # Get the appsink element and set the callback function
        appsink = self.pipeline.get_by_name("sink")
        if appsink:
            appsink.connect("new-sample", self.on_new_sample, self.frame_queue)
        else:
            print("Failed to get appsink element")

        # Set up the loop and bus for handling events
        bus = self.pipeline.get_bus()
        self.bus = bus
        bus.add_signal_watch()
        bus.connect("message::eos", self.on_eos)
        bus.connect("message::error", self.on_error)

        # Start the pipeline
        self.pipeline.set_state(Gst.State.PLAYING)

        # Start a separate thread for displaying frames
        # self.display_thread = threading.Thread(target=self.display_frames)
        # self.display_thread.start()

        try:
            self.loop.run()
        except:
            pass

        # Stop the pipeline when done
        self.pipeline.set_state(Gst.State.NULL)
        return self.data
    

def convert_to_dict(input_str):

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

    



if __name__ == "__main__":
    rtsp_url = "rtsp://10.0.0.222/color" 
    stream_processor = RTSPStreamProcessor(rtsp_url)
    data_str = stream_processor.start_rtsp_stream()
    data = convert_to_dict(data_str)
    print(data)
    print(type(data))

