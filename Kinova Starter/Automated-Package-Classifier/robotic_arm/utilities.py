import argparse
import yaml
import logging
import logging.config
from pathlib import Path

from kortex_api.TCPTransport import TCPTransport
from kortex_api.UDPTransport import UDPTransport
from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.messages import Session_pb2

BASE_DIR = Path(__file__).resolve().parent.parent  # Moves up one level
CONFIG_PATH = BASE_DIR / "config.yaml"
LOGGING_CONFIG_PATH = BASE_DIR / "logging.yaml"

if LOGGING_CONFIG_PATH.exists():
    with open(LOGGING_CONFIG_PATH, "r") as f:
        logging.config.dictConfig(yaml.safe_load(f))

logger = logging.getLogger(__name__)

def load_config():
    if CONFIG_PATH.exists():
        with open(CONFIG_PATH, "r") as f:
            return yaml.safe_load(f)
    else:
        logger.error(f"Configuration file not found: {CONFIG_PATH}")
        raise FileNotFoundError(f"Missing configuration file: {CONFIG_PATH}")

config = load_config()
logger.info("Loaded configuration file.")

def parseConnectionArguments(parser = argparse.ArgumentParser()):
    parser.add_argument("--ip", type=str, help="IP address of destination", default=config.get("local_robot_ip", "10.0.0.222"))
    parser.add_argument("-u", "--username", type=str, help="Username to login", default=config.get("user_login", "admin"))
    parser.add_argument("-p", "--password", type=str, help="Password to login", default=config.get("user_password", "admin"))
    return parser.parse_args()

class DeviceConnection:
    
    TCP_PORT = config.get("tcp_port", 10000)
    UDP_PORT = config.get("udp_port", 10001)

    @staticmethod
    def createTcpConnection(args): 
        """
        returns RouterClient required to create services and send requests to device or sub-devices,
        """
        logger.info(f"Creating TCP connection to {args.ip}:{DeviceConnection.TCP_PORT}")

        return DeviceConnection(args.ip, port=DeviceConnection.TCP_PORT, credentials=(args.username, args.password))

    @staticmethod
    def createUdpConnection(args): 
        """        
        returns RouterClient that allows to create services and send requests to a device or its sub-devices @ 1khz.
        """
        logger.info(f"Creating UDP connection to {args.ip}:{DeviceConnection.UDP_PORT}")

        return DeviceConnection(args.ip, port=DeviceConnection.UDP_PORT, credentials=(args.username, args.password))

    def __init__(self, ipAddress, port=TCP_PORT, credentials = ("","")):

        self.ipAddress = ipAddress
        self.port = port
        self.credentials = credentials

        self.sessionManager = None

        # Setup API
        self.transport = TCPTransport() if port == DeviceConnection.TCP_PORT else UDPTransport()
        self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)

        logger.info(f"Initializing connection to {self.ipAddress} on port {self.port}")

    # Called when entering 'with' statement
    def __enter__(self):
        
        self.transport.connect(self.ipAddress, self.port)
        logger.info(f"Connected to {self.ipAddress} on port {self.port}")

        if (self.credentials[0] != ""):
            session_info = Session_pb2.CreateSessionInfo()
            session_info.username = self.credentials[0]
            session_info.password = self.credentials[1]
            session_info.session_inactivity_timeout = 10000   # (milliseconds)
            session_info.connection_inactivity_timeout = 2000 # (milliseconds)

            self.sessionManager = SessionManager(self.router)
            logger.info(f"Logging in as {self.credentials[0]} on device {self.ipAddress}")
            self.sessionManager.CreateSession(session_info)

        return self.router

    # Called when exiting 'with' statement
    def __exit__(self, exc_type, exc_value, traceback):
    
        if self.sessionManager != None:

            router_options = RouterClientSendOptions()
            router_options.timeout_ms = 1000 
            
            self.sessionManager.CloseSession(router_options)
            logger.info("Session closed successfully.")

        self.transport.disconnect()
        logger.info(f"Disconnected from {self.ipAddress}.")
