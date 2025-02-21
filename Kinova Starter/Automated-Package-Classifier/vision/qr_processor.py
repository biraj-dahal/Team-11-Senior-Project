from dataclasses import dataclass
import json
import cv2
import numpy as np
from typing import Optional, Dict, Any, Tuple
import logging
import re

@dataclass
class QRData:
    package_type: str
    phone: str

class QRProcessor:
    def __init__(self):
        self.detector = cv2.QRCodeDetector()
        self.logger = logging.getLogger(__name__)

    def process_frame(self, frame: np.ndarray) -> Optional[QRData]:
        try:
            data, pts, qr_code = self.detector.detectAndDecode(frame)
            self.logger.info(f"QR data: {data}")
            if data:
                parsed_data = self._parse_qr_data(data)
                return QRData(**parsed_data)
            return None
        except Exception as e:
            self.logger.error(f"Error processing QR frame: {e}")
            return None

    def _parse_qr_data(self, data: str) -> Dict[str, Any]:
        try:
            corrected_str = re.sub(r'([a-zA-Z_][a-zA-Z0-9_]*)\s*:', r'"\1":', data)
            corrected_str = re.sub(r':\s*([a-zA-Z0-9_]+)(?=\s*[,}])', r':"\1"', corrected_str)
            
            return json.loads(corrected_str)
        except json.JSONDecodeError as e:
            self.logger.error(f"Error parsing QR data: {e}")
            raise ValueError("Invalid QR data format")