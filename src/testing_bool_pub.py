#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

class RosNode:
    def __init__(self):
        rospy.init_node("testBoolNode")
        pub_bool = rospy.Publisher("degen/is_degenerate", Bool, queue_size=10)
        
        msg = Bool()
        msg.data = True
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            pub_bool.publish(msg)
            rate.sleep()


if __name__ == "__main__":
    try:
        rosnode = RosNode()
    except rospy.ROSInterruptException:
        pass
