# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudPublisher(Node):

    def __init__(self):
        super().__init__('point_cloud_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'random_point_cloud', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.publish_point_cloud)

    def publish_point_cloud(self):
        msg = self.generate_random_point_cloud()
        self.publisher_.publish(msg)
        self.get_logger() . info("Publishing random point cloud")

    def generate_random_point_cloud(self , num_points = 1000):
       points = np.random.rand(num_points,3)
       header = Header()
       header.stamp = self.get_clock().now().to_msg()
       header.frame_id = "map"
       fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
       

       point_cloud = pc2.create_cloud(header,fields, points)
       return point_cloud

def main(args=None):
    rclpy.init(args=args)
    point_cloud_publisher = PointCloudPublisher()
    rclpy.spin(point_cloud_publisher)
    point_cloud_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
