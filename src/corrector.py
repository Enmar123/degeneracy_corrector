#!/usr/bin/env python

import numpy
import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class RosNode:
    def __init__(self):
        rospy.init_node("degeneracyCorrectorNode")
        rospy.loginfo("Degeneracy_Corrector: Initializing")
        
        # Get launch file params
        self.map_frame = rospy.get_param("~map_frame")
        self.odom_target_topic = rospy.get_param("~odom_target_topic")
        self.odom_degenerate_topic = rospy.get_param("~odom_degenerate_topic")
        self.bool_degeneracy_topic = rospy.get_param("~bool_degeneracy_topic")
        self.update_rate = rospy.get_param("~update_rate")
        # Set Class Params
        self.trans = [0,0,0]
        self.rot = [0,0,0]
        self.odom_target = Odometry()
        self.odom_degenerate = Odometry()
        # Configure ROS machines?
        rospy.loginfo("Degeneracy_Corrector: Configuring")
        self.br = tf.TransformBroadcaster()
        self.li = tf.TransformListener()
        self.sub_odom_target = rospy.Subscriber(self.odom_target_topic,
                                                Odometry,
                                                self.callbackTarget,
                                                queue_size=10)
        self.sub_odom_degen = rospy.Subscriber(self.odom_degenerate_topic,
                                               Odometry,
                                               self.callbackDegen,
                                               queue_size=10)
        self.sub_bool_is_degen = rospy.Subscriber(self.bool_degeneracy_topic,
                                                  Bool,
                                                  self.callbackBool,
                                                  queue_size=10)
        
        self.rate = rospy.Rate(self.update_rate)
        rospy.loginfo("Degeneracy_Corrector: Starting")
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
                              self.odom_degenerate.header.frame_id,
                              self.map_frame)
    
    def callbackBool(self, msg):
        if msg.data == True:
            while not rospy.is_shutdown():
                try:
                    (trans1,rot1) = self.li.lookupTransform(self.map_frame,
                                                            self.odom_target.child_frame_id,
                                                            rospy.Time(0))
                    (trans2,rot2) = self.li.lookupTransform(self.map_frame,
                                                            self.odom_degenerate.child_frame_id,
                                                            rospy.Time(0))
                    trans_update = [a_i - b_i for a_i, b_i in zip(trans1, trans2)]
                    rot_update   = [a_i - b_i for a_i, b_i in zip(rot1, rot2)]
                    self.trans   = [a_i + b_i for a_i, b_i in zip(self.trans, trans_update)]
                    self.rot     = [a_i + b_i for a_i, b_i in zip(self.rot, rot_update)]
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
