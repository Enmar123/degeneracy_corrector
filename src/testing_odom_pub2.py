#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry

class RosNode:
    def __init__(self):
        rospy.init_node("testOdomNode")
        pub_odom = rospy.Publisher("degen/odom", Odometry, queue_size=10)
        
        msg = Odometry()
        msg.header.frame_id = "odom_degen"
        msg.child_frame_id = "base_degen"
        msg.pose.pose.position.x = 0
        msg.pose.pose.position.y = 0.5
        
        quat = tf.transformations.quaternion_from_euler(0, 0, 0.753)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        rospy.loginfo("odom_pub2: %s"%(str(quat)))
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            msg.header.stamp = rospy.Time.now()
            pub_odom.publish(msg)
            rate.sleep()


if __name__ == "__main__":
    try:
        rosnode = RosNode()
    except rospy.ROSInterruptException:
        pass
