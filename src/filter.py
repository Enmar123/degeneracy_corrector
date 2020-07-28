#!/usr/bin/env python
"""
Created on Mon Jul 27 20:59:32 2020

@author: lattice
"""
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from pyquaternion import Quaternion
import copy

class RosNode:
    def __init__(self):
        rospy.init_node("DegenFilter")
        rospy.loginfo("DegenFilter: Initializing")
        
        rospy.Subscriber(rospy.get_param("~odom_ref"),
                         Odometry,
                         self.refOdomCallback,
                         queue_size=10)
        
        self.ref_odom = Odometry()
        
        self.odom_subs = []
        self.bool_subs = []
        self.odom_pubs = []
        self.degens = []
        self.delta_odoms = []
        i=0
        while not rospy.is_shutdown():
            try:
                odom_topic_in = rospy.get_param("~odom%d"%(i))
                degen_topic_in = odom_topic_in + "/is_degen"
                odom_topic_out = odom_topic_in + "_degenfilter"
                self.odom_subs.append(rospy.Subscriber(odom_topic_in,
                                                       Odometry,
                                                       self.odomCallback,
                                                       i,
                                                       queue_size=10))
                self.bool_subs.append(rospy.Subscriber(degen_topic_in,
                                                       Bool,
                                                       self.boolCallback,
                                                       i,
                                                       queue_size=10))
                self.odom_pubs.append(rospy.Publisher(odom_topic_out,
                                                       Odometry,
                                                       queue_size=10))
                self.degens.append(False)
                self.delta_odoms.append(Odometry())
                i += 1
            except:
                break
            
        rospy.loginfo("DegenFilter: Starting")
        rospy.spin()
        
    def refOdomCallback(self, msg):
        # if reference odom recieved, save msg for later acess
        self.ref_odom = msg
    
    def boolCallback(self, msg, i):
        if msg.data == True:
            self.degens[i] = True
        else:
            self.degens[i] = False
        
    def odomCallback(self, msg, i):
        if self.degens[i] == True:
            # if msg is degen, update delta and dont publish
            self.delta_odoms[i] = odomSubtractPose1from2(msg, self.ref_odom)
        else:
            # if msg not degen, add delta to msg and publish
            corrected_odom = odomAddPose1to2(self.delta_odoms[i], msg)
            self.odom_pubs[i].publish(corrected_odom)
        
def odomAddPose1to2(odom1, odom2):
    odom3 = copy.deepcopy(odom2)
    
    odom3.pose.pose.position.x = odom2.pose.pose.position.x + odom1.pose.pose.position.x
    odom3.pose.pose.position.y = odom2.pose.pose.position.y + odom1.pose.pose.position.y
    odom3.pose.pose.position.z = odom2.pose.pose.position.z + odom1.pose.pose.position.z
        
    o1 = odom1.pose.pose.orientation
    o2 = odom2.pose.pose.orientation
    
    q3 = Quaternion(o2.w+o1.w, o2.x+o1.x, o2.y+o1.y, o2.z+o1.z).normalised
    odom3.pose.pose.orientation.w = q3.elements[0]
    odom3.pose.pose.orientation.x = q3.elements[1]
    odom3.pose.pose.orientation.y = q3.elements[2]
    odom3.pose.pose.orientation.z = q3.elements[3]

    return odom3       

def odomSubtractPose1from2(odom1, odom2):
    odom3 = copy.deepcopy(odom2)
    
    odom3.pose.pose.position.x = odom2.pose.pose.position.x - odom1.pose.pose.position.x
    odom3.pose.pose.position.y = odom2.pose.pose.position.y - odom1.pose.pose.position.y
    odom3.pose.pose.position.z = odom2.pose.pose.position.z - odom1.pose.pose.position.z
    
    o1 = odom1.pose.pose.orientation
    o2 = odom2.pose.pose.orientation
    
    q3 = Quaternion(o2.w-o1.w, o2.x-o1.x, o2.y-o1.y, o2.z-o1.z).normalised
    odom3.pose.pose.orientation.w = q3.elements[0]
    odom3.pose.pose.orientation.x = q3.elements[1]
    odom3.pose.pose.orientation.y = q3.elements[2]
    odom3.pose.pose.orientation.z = q3.elements[3]
    
    return odom3            

if __name__ == "__main__":
    try:
        rosnode = RosNode()
    except rospy.ROSInterruptException:
        pass