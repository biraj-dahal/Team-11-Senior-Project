# abcd




#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import PointCloud2
from tf.transformations import quaternion_matrix
import numpy as np

def tf_callback(msg):
    # Extract the position and orientation
    position = msg.transform.translation
    orientation = msg.transform.rotation
    
    # Print position and orientation
    rospy.loginfo("Position: %s", position)
    rospy.loginfo("Orientation: %s", orientation)
    
    # Calculate an offset (example values, replace with your desired offset)
    offset = np.array([0.1, 0.1, 0.1, 1])  # Offset in meters (x, y, z, 1)
    rotated_offset = np.dot(quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w]), offset)[:3]
    offset_position = np.array([position.x, position.y, position.z]) + rotated_offset
    
    # Publish point cloud with the calculated offset
    publish_point_cloud(offset_position)

def publish_point_cloud(offset_position):
    pub = rospy.Publisher('/point_cloud', PointCloud2, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        # Create a PointCloud2 message
        pc_msg = PointCloud2()
        # Fill in details of pc_msg based on your point cloud data and offset_position
        
        # Publish the message
        pub.publish(pc_msg)
        
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('tf_listener')
    listener = tf.TransformListener()
    
    rospy.Subscriber("/tf", tf.msg.tfMessage, tf_callback)
    
    rospy.spin()







import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def point_cloud_callback(msg):
    points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    points_lists = list(points)
    rospy.loginfo(f"Received point cloud with {len(points_lists)} points")
    for point in points_lists[:10]:
        rospy.loginfo(f"Point: x={point[0]}, y={point[1]}, z={point[2]}")

    # Republish the received PointCloud2 message
    new_point_cloud_publisher.publish(msg)

def listener():
    global new_point_cloud_publisher
    
    rospy.init_node("point_cloud_subscriber", anonymous=True)
    
    # Initialize the publisher
    new_point_cloud_publisher = rospy.Publisher("new_point_cloud_topic", PointCloud2, queue_size=10)
    
    rospy.Subscriber("random_point_cloud", PointCloud2, point_cloud_callback)
    rospy.spin()

if __name__ == "__main__":
    bridge = CvBridge()
    listener()
