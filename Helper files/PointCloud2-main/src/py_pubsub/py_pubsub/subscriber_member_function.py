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
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class PointCloudSubscriber(Node):

    def __init__(self):
        super().__init__('point_cloud_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            'random_point_cloud',
            self.listener_callback,
            10)
        self.subscription 
        

# class MinimalSubscriber(Node):

#     def __init__(self):
#         super().__init__('minimal_subscriber')
#         self.string_subscription = self.create_subscription(
#             String,
#             'topic',
#             self.listener_callback,
#             10)
#         self.string_subscription 
    



    def listener_callback(self, msg):
        points = pc2.read_points(msg, field_names= ("x","y","z"), skip_nans= True)
        points_list = list(points)
        self.get_logger().info(f"Recieved point cloud with {len(points_list)} points")
        for point in points_list[:10]:
            self.get_logger().info(f"Point: x={point[0]}, y= {point[1]} , z={point[2]}")    
    # def image_callback(self,msg):
    #     try:
    #         cv_image = self.br.imgmsg_to_cv2(msg , desired_encoding= "bgr8")
    #         self.grt_logger().info("recieved" % (msg.width, msg.height))
    #     except CvBridgeError as e:
    #         self.get_logger().error("failed: %s" % str(e))


def main(args=None):
    rclpy.init(args=args)
    point_cloud_subscriber = PointCloudSubscriber()
    rclpy.spin(point_cloud_subscriber)
    point_cloud_subscriber.destroy_node()
    rclpy.shutdown()

    #minimal_subscriber = MinimalSubscriber()

    #rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # minimal_subscriber.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
