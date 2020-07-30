#!/usr/bin/env python

import numpy as np
from scipy import stats 

import rospy
import message_filters
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool



class RosNode:
    def __init__(self):
        rospy.init_node("DegenDetector")
        rospy.loginfo("DegenDetector: Initializing")
        
        # Get params
        self.threshold = rospy.get_param("~degen_threshold")
        self.config = rospy.get_param("~odom_config")
        
        
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
        ts = message_filters.ApproximateTimeSynchronizer(self.odom_subs, 10, 0.1)
        ts.registerCallback(self.detectorCallback)
    
        rospy.loginfo("DegenDetector: Starting")
        rospy.spin()
        
    def detectorCallback(self, *args):
        # Compare velocities for each velocity vector component sequentially
        # to attemt and detect degeneracy.
        odoms = list(args)        
        datas = self.makeDatas(odoms)
        for data in datas:
            velocitys = data[0]
            variances = data[1]
            index = self.getVelocityOutlierIndex(velocitys, variances)
            if index != None:
                self.pubDegeneracyForIndex(index)
                return
        self.pubDegeneracyForIndex(None)
    
    def makeDatas(self, odoms):
        # Formats data to use for degeneracy detection
        datas = []
        if self.config[0]:
            velocitys = [odom.twist.twist.linear.x for odom in odoms]
            variances = [odom.twist.covariance[0*7] for odom in odoms]
            datas.append((velocitys,variances))
        if self.config[1]:
            velocitys = [odom.twist.twist.linear.y for odom in odoms]
            variances = [odom.twist.covariance[1*7] for odom in odoms]
            datas.append((velocitys,variances))
        if self.config[2]:
            velocitys = [odom.twist.twist.linear.z for odom in odoms]
            variances = [odom.twist.covariance[2*7] for odom in odoms]
            datas.append((velocitys,variances))
        if self.config[3]:
            velocitys = [odom.twist.twist.angular.x for odom in odoms]
            variances = [odom.twist.covariance[3*7] for odom in odoms]
            datas.append((velocitys,variances))
        if self.config[4]:
            velocitys = [odom.twist.twist.angular.y for odom in odoms]
            variances = [odom.twist.covariance[4*7] for odom in odoms]
            datas.append((velocitys,variances))
        if self.config[5]:
            velocitys = [odom.twist.twist.angular.z for odom in odoms]
            variances = [odom.twist.covariance[5*7] for odom in odoms]
            datas.append((velocitys,variances))
        return datas
            
    
    def getVelocityOutlierIndex(self, velocities, variances):
        # The likely outlier must deviate by some percentage and must also
        # fall outside the variance-based standard deviation before being
        # considered degenerate.
        stdevs = [ variance**(1/2) for variance in variances ]
        index = getLikelyOutlierIndex(velocities)
        if all([isIndexPercentOutlier(index, velocities, self.threshold),
                isIndexStdevOutlier(index, velocities, stdevs)]):
            return index
        else:
            return None
    
    def pubDegeneracyForIndex(self, index=None):
        i = 0
        for bool_pub in self.bool_pubs:
            if i == index:
                bool_pub.publish(self.msg_bool_true)
            else:
                bool_pub.publish(self.msg_bool_false)
            i += 1

def getLikelyOutlierIndex(mylist):
    scores = list(abs(stats.zscore(mylist)))
    likely_outlier_index = scores.index(max(scores))
    return likely_outlier_index

def isIndexPercentOutlier(index, mylist, threshold):
    others = getOthers(index, mylist)
    others_avg = np.mean(others)
    if others_avg == 0: # mitigate division problems with zeros
        error = 100.0   # defaults to high error
    else:
        error = abs((others_avg - mylist[index])/others_avg)
    
    if (error >= threshold):
        return True
    else:
        return False
    
def isIndexStdevOutlier(index, mylist, stdevs):
    target = mylist[index]
    targetstdev = stdevs[index]
    samplelist = np.array(getOthers(index, mylist))
    samplestdev = np.array(getOthers(index, stdevs))
    # Test if the target falls within the stdev of all othe points
    maxbar = max(samplelist + samplestdev)
    minbar = min(samplelist - samplestdev)
    if (minbar <= target <= maxbar):
        return False
    # test if any points fall within the stdev of the target 
    maxbar = target + targetstdev
    minbar = target - targetstdev
    for sample in samplelist:
        if (minbar <= sample <= maxbar):
            return False
    # If no stdev overlap present then it is an outlier
    return True
      
def getOthers(index, mylist):
    i = 0
    others = []
    for item in mylist:
        if index != i :
            others.append(item)
        i+=1
    return others

if __name__ == "__main__":
    try:
        rosnode = RosNode()
    except rospy.ROSInterruptException:
        pass
