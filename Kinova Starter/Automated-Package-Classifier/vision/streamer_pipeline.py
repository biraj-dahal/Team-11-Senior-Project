import gi
import numpy as np
import cv2
import threading
from queue import Queue
from gi.repository import Gst, GLib
import logging
from dataclasses import dataclass
from typing import Optional
from vision.qr_processor import QRProcessor
import json

gi.require_version('Gst', '1.0')

@dataclass
class StreamConfig:
    width: int
    height: int
    format: str

class RTSPStreamProcessor:
    def __init__(self, rtsp_url: str):
        """ Initialize the RTSP stream processor with the RTSP URL """
        self.rtsp_url = rtsp_url
        self.frame_queue = Queue()
        self.loop = GLib.MainLoop()
        self.pipeline = None
        self.logger = logging.getLogger(__name__)
        self.bus = ""
        self.data = ""
        self.current_frame = None
        self._stream_lock = threading.Lock()
        self._is_running = False
        self.qr_processor = QRProcessor()

    def on_eos(self, bus, message):
        """ End of stream callback """
        self.logger.info('End of Stream received')
        self._is_running = False
        self.loop.quit()

    def on_error(self, bus, message):
        """ Error callback with detailed logging """
        err, debug_info = message.parse_error()
        self.logger.error(f"Stream Error: {err}, Debug: {debug_info}")
        self._is_running = False
        self.loop.quit()

    def on_new_sample(self, sink, user_data):
        """ Callback function to grab a frame from the pipeline and detect QR codes """
        try:
                sample = sink.emit("pull-sample")
                if not sample:
                    return Gst.FlowReturn.ERROR
                buffer = sample.get_buffer()
                caps = sample.get_caps()
                structure = caps.get_structure(0)

                config = StreamConfig(
                    width=structure.get_value('width'),
                    height=structure.get_value('height'),
                    format=structure.get_value('format')
                )

                success, map_info = buffer.map(Gst.MapFlags.READ)
                if not success:
                    return Gst.FlowReturn.ERROR

                try:
                    frame = np.ndarray(
                        (config.height, config.width, 3),
                        dtype=np.uint8,
                        buffer=map_info.data
                    )
                    self.current_frame = frame.copy()

                    qr_data = self.qr_processor.process_frame(frame)
                    if qr_data:
                        self.logger.info(f"QR Code Data detected: {qr_data}")
                        self.data = json.dumps({
                            "package_type": qr_data.package_type.lower(),  
                            "phone": qr_data.phone
                        })
                        self.on_eos(bus=self.bus, message="Data detected")
                    
                    user_data.put(frame)

                finally:
                    buffer.unmap(map_info)

                return Gst.FlowReturn.OK

        except Exception as e:
  
                self.logger.error(f"Error processing sample: {e}")
                return Gst.FlowReturn.ERROR

    def setup_pipeline(self) -> bool:
        try:
            Gst.init(None)
            pipeline_str = (
                f"rtspsrc location={self.rtsp_url} ! "
                "decodebin ! "
                "videoconvert ! "
                "video/x-raw, format=RGB ! "
                "appsink name=sink emit-signals=True"
            )
            self.pipeline = Gst.parse_launch(pipeline_str)
            
            appsink = self.pipeline.get_by_name("sink")
            if not appsink:
                raise RuntimeError("Failed to create appsink")
            
            appsink.connect("new-sample", self.on_new_sample, self.frame_queue)
            
            self.bus = self.pipeline.get_bus()
            self.bus.add_signal_watch()
            self.bus.connect("message::eos", self.on_eos)
            self.bus.connect("message::error", self.on_error)
            
            return True
        
        except Exception as e:
            self.logger.error(f"Failed to setup pipeline: {e}")
            return False

    def start_rtsp_stream(self) -> str:
        """ Start the RTSP stream and return the QR code data when found """
        try:
            self.logger.info("Reached start_rtsp_stream")
            if not self.setup_pipeline():
                return ""
            self.logger.info("Reached start_rtsp_stream2")
            self._is_running = True
            self.pipeline.set_state(Gst.State.PLAYING)
            self.logger.info("Reached start_rtsp_stream3")

            try:
                self.logger.info("Reached start_rtsp_stream4")
                self.loop.run()
            except Exception as e:
                self.logger.error(f"Error in main loop: {e}")
                return ""
            
            self.logger.info("Reached start_rtsp_stream5")

            self.pipeline.set_state(Gst.State.NULL)
            return self.data

        except Exception as e:
            self.logger.error(f"Failed to start stream: {e}")
            return ""

    def get_current_frame(self) -> Optional[np.ndarray]:
        """Return a copy of the current frame with thread safety"""
        with self._stream_lock:
            return self.current_frame.copy() if self.current_frame is not None else None

    def stop(self) -> None:
        """Stop the RTSP stream processing"""
        self._is_running = False
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        if self.loop.is_running():
            self.loop.quit()