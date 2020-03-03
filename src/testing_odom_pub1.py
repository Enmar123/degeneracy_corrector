#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

class RosNode:
    def __init__(self):
        rospy.init_node("testOdomNode")
        pub_odom = rospy.Publisher("target/odom", Odometry, queue_size=10)
        
        msg = Odometry()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base"
        msg.pose.pose.position.x = 1
        msg.pose.pose.position.y = 1
	msg.pose.pose.orientation.w = 1
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            msg.header.stamp = rospy.Time.now()
            pub_odom.publish(msg)
            msg.pose.pose.position.x += 0.01
            rate.sleep()


if __name__ == "__main__":
    try:
        rosnode = RosNode()
    except rospy.ROSInterruptException:
        pass
