#!/usr/bin/env python

import numpy
import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class RosNode:
    def __init__(self):
        rospy.init_node("degeneracyCorrectorNode")
        
        # Get launch file params
#        odom_target = rospy.get_param("~odom_target")
#        odom_degenertate = rospy.get_param("~odom_degenerate")
#        bool_is_degenerate = rospy.get_param("~is_degenerate")
        # Set Class Params
        self.trans = [0,0,0]
        self.rot = [0,0,0]
        self.odom_target = None
        self.odom_degenerate = None
        # Configure ROS machines?
        self.br = tf.TransformBroadcaster()
        self.li = tf.TransformListener()
        self.sub_odom_target = rospy.Subscriber("target/odom",
                                                Odometry,
                                                self.callbackTarget,
                                                queue_size=10)
        self.sub_odom_degen = rospy.Subscriber("degen/odom",
                                               Odometry,
                                               self.callbackDegen,
                                               queue_size=10)
        self.sub_bool_is_degen = rospy.Subscriber("degen/is_degenerate",
                                                  Bool,
                                                  self.callbackBool,
                                                  queue_size=10)
        
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.setTf()
            #rospy.loginfo("degenCorrectorNode: %s"%(str(self.pos)))
            self.rate.sleep()

    def setTf(self):
        self.br.sendTransform(self.trans,
                              tf.transformations.quaternion_from_euler(self.rot[0],
                                                                       self.rot[1],
                                                                       self.rot[2]),
                              rospy.Time.now(),
                              "odom_degen",
                              "map")
    
    def callbackBool(self, msg):
        if msg.data == True:
            while not rospy.is_shutdown():
                try:
                    #(trans,rot) = self.li.lookupTransform('base_degen', 'base', rospy.Time(0))
                    (trans1,rot1) = self.li.lookupTransform('map', 'base', rospy.Time(0))
                    (trans2,rot2) = self.li.lookupTransform('map', 'base_degen', rospy.Time(0))
                    trans = [a_i - b_i for a_i, b_i in zip(trans1, trans2)]
                    rot   = [a_i - b_i for a_i, b_i in zip(rot1, rot2)]
                    self.trans = [a_i + b_i for a_i, b_i in zip(self.trans, trans)]
                    self.rot   = [a_i + b_i for a_i, b_i in zip(self.rot, rot)]
                    rospy.loginfo("degenCorrectorNode: %s"%(str(self.trans)))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    self.rate.sleep()
                    continue
                break
            pass
            
            
            # compute and transform
    def callbackDegen(self, msg):
        self.odom_degenerate = msg
    
    def callbackTarget(self, msg):
        self.odom_target = msg


if __name__ == "__main__":
    try:
        rosnode = RosNode()
    except rospy.ROSInterruptException:
        pass
