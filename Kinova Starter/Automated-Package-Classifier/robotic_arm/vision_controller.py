import logging
import re
import json
import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from vision.streamer_pipeline import RTSPStreamProcessor

class VisionController:
    def __init__(self, rtsp_url: str):
        self.rtsp_url = rtsp_url
        self.logger = logging.getLogger(__name__)

    def scan_qr(self):
        try:
            stream_processor = RTSPStreamProcessor(self.rtsp_url)
            data_str = stream_processor.start_rtsp_stream()
            
            if not data_str:
                self.logger.info("No QR code data received")
                return None
            
            try:
                data = json.loads(data_str)
                if not isinstance(data, dict) or 'package_type' not in data:
                    self.logger.info("Invalid QR data format")
                    return None
                return data
            except json.JSONDecodeError as e:
                self.logger.info(f"Failed to parse QR data: {e}")
                return None
            
        except Exception as e:
            self.logger.error(f"Error in QR scanning: {e}")
            return None
