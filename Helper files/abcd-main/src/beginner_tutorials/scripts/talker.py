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

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from sensor_msgs.msg import PointCloud2 , PointField
from std_msgs.msg import Header
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

def talker():
    pub = rospy.Publisher('random_point_cloud', PointCloud2, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        point_cloud_msg = generate_random_point_cloud()
        rospy.loginfo("Publishing random point cloud")
        pub.publish(point_cloud_msg)
        rate.sleep()

def generate_random_point_cloud(num_points=1000):
    points = np.random.rand(num_points,3)
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "map"
    fields = [
        PointField(name ="x" , offset=0, datatype = PointField.FLOAT32, count=1),
        PointField(name ="y" , offset=4, datatype = PointField.FLOAT32, count=1),
        PointField(name ="z" , offset=8, datatype = PointField.FLOAT32, count=1)
    ]

    point_cloud = pc2.create_cloud(header, fields, points)
    return point_cloud
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
