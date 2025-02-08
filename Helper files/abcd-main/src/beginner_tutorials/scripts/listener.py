#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

# import rospy
# from std_msgs.msg import String

# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

# def listener():

#     # In ROS, nodes are uniquely named. If two nodes with the same
#     # name are launched, the previous one is kicked off. The
#     # anonymous=True flag means that rospy will choose a unique
#     # name for our 'listener' node so that multiple listeners can
#     # run simultaneously.
#     rospy.init_node('listener', anonymous=True)

#     rospy.Subscriber('chatter', String, callback)

#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()

# if __name__ == '__main__':
#     listener()




import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def point_cloud_callback(msg):
    points = pc2.read_points(msg, field_names=("x", "y","z"), skip_nans=True)
    points_lists = list(points)
    rospy.loginfo(f"Received points cloud with{len(points_lists)} points")
    for point in points_lists[:10]:
        rospy.loginfo(f"Point: x={point[0]}, y={point[1]},z= {point[2]}")

def listener():
    rospy.init_node("point_cloud_subscriber" ,anonymous = True)
    rospy.Subscriber("random_point_cloud",PointCloud2, point_cloud_callback)
    rospy.spin()


if __name__ == "__main__":
    bridge = CvBridge()
    listener()



# import rospy
# from sensor_msgs.msg import PointCloud2
# import sensor_msgs_py.point_cloud2 as pc2
# from std_msgs.msg import String
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError


# def point_cloud_callback(msg):
#     points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
#     points_lists = list(points)
#     rospy.loginfo(f"Received point cloud with {len(points_lists)} points")
#     for point in points_lists[:10]:
#         rospy.loginfo(f"Point: x={point[0]}, y={point[1]}, z={point[2]}")

#     # Republish the received PointCloud2 message
#     new_point_cloud_publisher.publish(msg)

# def listener():
#     global new_point_cloud_publisher
    
#     rospy.init_node("point_cloud_subscriber", anonymous=True)
    
#     # Initialize the publisher
#     new_point_cloud_publisher = rospy.Publisher("new_point_cloud_topic", PointCloud2, queue_size=10)
    
#     rospy.Subscriber("random_point_cloud", PointCloud2, point_cloud_callback)
#     rospy.spin()

# if __name__ == "__main__":
#     bridge = CvBridge()
#     listener()

    
