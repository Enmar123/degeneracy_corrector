#!/usr/bin/env python

import numpy as np

import rospy
import message_filters
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool



class RosNode:
    def __init__(self):
        rospy.init_node("DegenDetector")
        rospy.loginfo("DegenDetector: Initializing")
        
        # Get launch file params
#        self.map_frame = rospy.get_param("~map_frame")
#        self.odom_target_topic = rospy.get_param("~odom_target_topic")
#        self.odom_degenerate_topic = rospy.get_param("~odom_degenerate_topic")
#        self.bool_degeneracy_topic = rospy.get_param("~bool_degeneracy_topic")
#        self.update_rate = rospy.get_param("~update_rate")
        
        # Set Class Params
        self.msg_true = Bool()
        self.msg_true.data = True
        self.msg_false = Bool()
        self.msg_true.data = False
        self.odom_subs = []
        self.bool_pubs = []
        i = 0
        while not rospy.is_shutdown():
            try:
                odom_name = rospy.get_param("~odom%d"%(i))
                bool_name = odom_name + "/is_degen"
                self.odom_subs.append(message_filters.Subscriber(odom_name,
                                                                 Odometry))
                self.bool_pubs.append(rospy.Publisher(bool_name,
                                                      Bool))
                i += 1
            except:
                break
                    
        # Configure ROS
        rospy.loginfo("Comparative_Degeneracy: Configuring")
        
        ts = message_filters.ApproximateTimeSynchronizer(self.odom_subs, 10, 0.1)
        ts.registerCallback(self.callback)
    
        rospy.loginfo("Comparative_Degeneracy: Starting")
        rospy.spin()
        
    def callback(self, *args):
        msgs = list(args) #list of odometry msgs
        tests = [self.is_targetVelocityAnOutlierInX]
        i = 0
        for msg in msgs:
            target = msg
            others = getOthers(i, msgs)
            for test in tests:
                if test(target, others) is True:
                    self.pubDegeneracy(i)
                    return
        self.pubDegeneracy(None)
    
    def is_targetVelocityAnOutlierInX(self, target_msg, other_msgs):
        error = 0.25
        target_vx = target_msg.twist.twist.linear.x
        for msg in other_msgs:
            other_vx = msg.twist.twist.linear.x
            other_vx_var = msg.twist.covariance[0]
            stdev = np.sqrt(other_vx_var) 
            if abs(target_vx - other_vx)/target_vx < error:
                return False
            if (other_vx - stdev) < target_vx < (other_vx + stdev):    
                return False
        return True 

        pass
    
    def pubDegeneracy(self, index=None):
        i = 0
        for bool_pub in self.bool_pubs:
            if i == index:
                bool_pub.publish(self.msg_bool_true)
            else:
                bool_pub.publish(self.msg_bool_false)
            i += 1

        
def getOthers(index, mylist):
    i = 0
    others = []
    for item in mylist:
        if index != i :
            others.append(item)
    return others

if __name__ == "__main__":
    try:
        rosnode = RosNode()
    except rospy.ROSInterruptException:
        pass
