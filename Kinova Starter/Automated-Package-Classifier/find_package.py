import sys
import os
import time
import logging
from robotic_arm.robot import Robot
import argparse

import robotic_arm.utilities as utilities
parser = argparse.ArgumentParser()
args = utilities.parseConnectionArguments(parser)

import logging.config
import yaml

def setup_logging():
    with open('cartesian_log.yaml', 'r') as f:
        config = yaml.safe_load(f)
    logging.config.dictConfig(config)
    return logging.getLogger(__name__)

logger = setup_logging()

from ultralytics import YOLO
from PIL import Image
import cv2
import numpy as np

MAX_X, MAX_Y = 900, 1000

def find_package(frame, model, min_confidence):
    """Locating the package to pick up"""
    
    # Perform inference on the frame
    results = model.predict(source=frame, show=True)
    print("Inference results received.")

    result = results[0]
    
    # Extract bounding boxes, class ids, and confidence scores from the result
    boxes = result.boxes.xyxy.cpu().numpy()  # Bounding box coordinates
    confidences = result.boxes.conf.cpu().numpy()  # Confidence scores
    class_ids = result.boxes.cls.cpu().numpy().astype(int)  # Class ids for each detection
    class_names = result.names

    center = ()

    for box, confidence, class_id in zip(boxes, confidences, class_ids):
        x_min, y_min, x_max, y_max = box
        if class_names[class_id] == 'book':
            logger.info(f"Package found! Here are the max dims: x-max ->{x_max} y-max ->{y_max}")
        if confidence >= min_confidence and x_max<MAX_X and y_max<MAX_Y:  # Threshold and dimesnions for confidence

            # Calculate the center
            x_center = (x_min + x_max) / 2
            y_center = (y_min + y_max) / 2
            
            #Remove if not logging:
            print(f"Detected Distinct Object: {class_names[class_id]}, confidence: {confidence}")

            # Check if the detected object is a "book" (class name matching 'book')
            if class_names[class_id] == 'book':  # Ensure 'book' is the correct class
                center = (y_center, x_center) # Store the center coordinates

                # Annotate the frame (drawing bounding box and center point)
                cv2.circle(frame, (int(x_center), int(y_center)), 5, (0, 255, 0), -1)  # Draw center
                cv2.rectangle(frame, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0, 255, 0), 2)  # Draw bounding box
                break

    return center, frame

def convert_to_cartesian(center, img_width=1920, img_height=1080, 
                         max_workspace_x=1, max_workspace_y=1):
    """
    Convert 2D screen coordinates (center of detected object) to Kinova Cartesian coordinates
    relative to the Kinova arm's workspace.
    """
    #TODO: Improve cartesian triangulation code 
    y_center, x_center = center  # Get the screen coordinates

    # Normalize the x and y coordinates
    x_norm = (x_center - (img_width//2)) / img_width
    y_norm = (y_center - (img_height//2)) / img_height

    # Scale normalized coordinates to the arm's workspace
    arm_x = x_norm * max_workspace_x
    arm_y = y_norm * max_workspace_y

    return [arm_x, arm_y]

def find_package_location(model):
    cap = cv2.VideoCapture("rtsp://10.0.0.222/color")
    if not cap.isOpened():
        print("Error: Couldn't open the RTSP stream.")
        return
    
    frame_count = 0
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Error: Couldn't read frame.")
            break

        # Find package (QR) and get annotated frame with center
        center, annotated_frame = find_package(frame, model, 0.25)

        # Save every frame with detected package
        if center:
            cv2.imwrite(f"frame_{frame_count}.jpg", annotated_frame)  # Save frame as .jpg
            logger.info(f"Detected center: {center}")
            return center

        # Press 'q' to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


        frame_count += 1

    cap.release()
    cv2.destroyAllWindows()

# Initialize the YOLO model
model = YOLO("yolov8m.pt")

vertical_descent = 0.22

with utilities.DeviceConnection.createTcpConnection(args) as router:
        robot = Robot(router)

        number_of_packages = 2
        while number_of_packages>0:
            try:
                # Move to Automated staging area
                robot.go_to_automated_home()
                if not robot.close_gripper_with_speed():
                    logger.error("Failed to open gripper")
                    break
                time.sleep(2)
                if number_of_packages==1:
                    break

                # Start processing the RTSP stream
                pos = find_package_location(model)
                adjust_coor = convert_to_cartesian(pos, 1920, 1080, 0.15, 0.15)
                logger.info(f"Normalized x,y: {adjust_coor}")

                target_coor = robot.get_cartesian_pose()
                logger.info(f"Cartesian coordinates: {target_coor}")

                #Triangulating x, y, and z
                target_coor[0]-=adjust_coor[0]
                target_coor[1]-=adjust_coor[1]
                target_coor[2]-=vertical_descent
                logger.info(f"Target is: {target_coor}")

                robot.go_to_cartesian(target_coor)
                time.sleep(2)
                logger.info(f"New coordinates: {robot.get_cartesian_pose()}")

                #TODO: connect pipeline to QR code flow 

            except Exception as e:
                logger.error(f"Error occurred: {e}")
            number_of_packages-=1
