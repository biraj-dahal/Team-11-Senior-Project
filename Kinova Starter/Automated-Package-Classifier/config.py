from dataclasses import dataclass
from typing import List
import yaml
import logging
from pathlib import Path

@dataclass
class RobotConfig:
    rtsp_url: str
    tcp_port: int
    udp_port: int
    timeout_duration: int
    default_gripper_speed: float
    
    @staticmethod
    def from_yaml(file_path: str) -> 'RobotConfig':
        with open(file_path, 'r') as f:
            config = yaml.safe_load(f)
        return RobotConfig(**config)

@dataclass
class Position:
    angles: List[float]
    name: str
    description: str

class PositionRegistry:
    def __init__(self):
        self.positions = {
            'PERISHABLE': Position([292.318, 64.931, 178.571, 253.963, 36.948, 348.73, 344.96], 
                                 'PERISHABLE', 'Position for perishable items'),
            'HAZARDOUS': Position([165.682, 358.721, 174.79, 260.499, 64.21, 329.353, 43.232],
                                'HAZARDOUS', 'Position for hazardous items'),
            'RETURN': Position([131.384, 34.513, 126.049, 232.762, 53.654, 316.227, 60.587],
                             'RETURN', 'Position for return items'),
            'MAINCONVEYER': Position([345.845, 39.657, 192.208, 250.67, 345.695, 331.633, 97.461],
                                   'MAINCONVEYER', 'Main conveyer position'),
            'QRSCAN': Position([342.552, 27.898, 196.617, 256.042, 350.27, 314.503, 93.662],
                             'QRSCAN', 'QR code scanning position')
        }

    def get_position(self, position_name: str) -> Position:
        if position_name not in self.positions:
            raise ValueError(f"Unknown position: {position_name}")
        return self.positions[position_name]