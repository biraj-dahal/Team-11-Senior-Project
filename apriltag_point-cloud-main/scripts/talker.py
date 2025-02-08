

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic
 

import rospy
from sensor_msgs.msg import PointCloud2 , PointField
from std_msgs.msg import Header , Bool
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2


apriltag_detection_status = False

def detect_apriltag():
    global apriltag_detection_status

    return apriltag_detection_status


def april_tag_callback(status_msg):
    global apriltag_detection_status
    apriltag_detection_status = status_msg.data
    rospy.loginfo(f"detection {apriltag_detection_status}")


def talker():
    global apriltag_detection_status
    """
    Main function to initialize ROS node, subscribe to AprilTag status, and publish random PointCloud2 messages.
    """
    print("Is it working?")

    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('random_point_cloud', PointCloud2, queue_size=10)
    rospy.Subscriber('apriltag_status', Bool, april_tag_callback)


    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if detect_apriltag() is True:
            point_cloud_msg = generate_random_point_cloud()
            rospy.loginfo("Publishing random point cloud")
            pub.publish(point_cloud_msg)
        else:
            point_cloud_msg = generate_empty_cloud()
            rospy.loginfo("Tag not detected")
            pub.publish(point_cloud_msg)
        rate.sleep()
        # point_cloud_msg = generate_random_point_cloud()
        # rospy.loginfo("Publishing random point cloud")
        # pub.publish(point_cloud_msg)
        # rate.sleep()


def generate_empty_cloud(num_points=0):
    points = np.random.rand(num_points,3)
    points = points * 0.1
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "tag_2"
    fields = [
        PointField(name ="x" , offset=0, datatype = PointField.FLOAT32, count=1),
        PointField(name ="y" , offset=4, datatype = PointField.FLOAT32, count=1),
        PointField(name ="z" , offset=8, datatype = PointField.FLOAT32, count=1)
    ]
    point_cloud = pc2.create_cloud(header, fields, points)
    return point_cloud


def generate_random_point_cloud(num_points=1000):
    points = np.random.rand(num_points,3)
    points = points * 0.1
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "tag_2"
    fields = [
        PointField(name ="x" , offset=0, datatype = PointField.FLOAT32, count=1),
        PointField(name ="y" , offset=4, datatype = PointField.FLOAT32, count=1),
        PointField(name ="z" , offset=8, datatype = PointField.FLOAT32, count=1)
    ]

    print(header.frame_id)

    point_cloud = pc2.create_cloud(header, fields, points)
    return point_cloud


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass