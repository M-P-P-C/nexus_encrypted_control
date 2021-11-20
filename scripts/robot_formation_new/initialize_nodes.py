#!/usr/bin/env python

import sys

import rospy
from std_msgs.msg import String

import numpy as np

from pymomorphic import pymorph


def nodes_init():
    
    rospy.init_node('dataprocessingnode_'+str(int(sys.argv[1])), anonymous=False)

    rospy.init_node('enc_'+str(int(sys.argv[1])), anonymous=False)

    rospy.init_node('controller_'+str(int(sys.argv[1])), anonymous=False)

    rospy.init_node('dec_'+str(int(sys.argv[1])), anonymous=False)
    
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        nodes_init()
        rospy.loginfo("Initialized nodes for Nexus_"+str(int(sys.argv[1]))+"...")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass