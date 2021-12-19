#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Point,Pose,PoseWithCovariance,PoseWithCovarianceStamped
import Utils

def send_init_pose(pub_init_pose, init_pose):
    pose_data = init_pose.split(",")
    assert len(pose_data) == 3

    x, y, theta = float(pose_data[0]), float(pose_data[1]), float(pose_data[2])
    q = Utils.angle_to_quaternion(theta)
    point = Point(x=x, y=y)
    pose = PoseWithCovariance(pose=Pose(position=point, orientation=q))
    pub_init_pose.publish(PoseWithCovarianceStamped(pose=pose))

if __name__ == "__main__":
    rospy.init_node("path_publisher")

    init_pose_topic = rospy.get_param("~init_pose_topic", "/initialpose")
    pub_init_pose = rospy.Publisher(init_pose_topic, PoseWithCovarianceStamped, queue_size=1)
    init_pose="50,12,-0.5"

    rospy.sleep(1.0)
    send_init_pose(pub_init_pose, init_pose)
