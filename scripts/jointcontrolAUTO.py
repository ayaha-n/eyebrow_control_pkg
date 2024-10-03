#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import numpy as np
import random

rospy.init_node('joint_control_cui', anonymous=True)


pub = rospy.Publisher('/joint_states', JointState, queue_size=10)


joints = [[100, 80, 45],
          [83, 90, 30],
          [120, 130, 10],
          [83, 90, 30],
          [115, 110, 85]]

rate = rospy.Rate(1)
i = 0
while not rospy.is_shutdown():
    rospy.loginfo('publish')
    js = JointState()
    js.name = ["servo1_joint", "servo2_joint", "servo3_joint"]
    js.position = joints[i]
    js.header.stamp = rospy.Time.now()
    pub.publish(js)
    rospy.sleep(3.0)
    i = (i + 1) % len(joints)
