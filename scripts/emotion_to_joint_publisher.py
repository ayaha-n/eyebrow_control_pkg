#!/usr/bin/env python3
#this program reads msg on /eyebrow_status and publishes /joint_states
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String  # /eyebrow_statusはString型と仮定
import numpy as np

# Joint angles for different emotions
emotion_joints = {
    "normal": [100, 80, 45],
    "happy": [83, 90, 30],
    "sad": [120, 130, 10],
    "angry": [115, 110, 85],
    "surprised": [83, 90, 30]
}

# Publisher
pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

# Callback function to process received /eyebrow_status messages
def eyebrow_status_callback(msg):
    emotion = msg.data
    if emotion in emotion_joints:
        rospy.loginfo(f'Received emotion: {emotion}')
        js = JointState()
        js.name = ["servo1_joint", "servo2_joint", "servo3_joint"]
        js.position = emotion_joints[emotion]
        js.header.stamp = rospy.Time.now()
        pub.publish(js)
        rospy.loginfo(f'Published joint states for: {emotion}')
    else:
        rospy.logwarn(f'Unknown emotion received: {emotion}')

def joint_control():
    rospy.init_node('joint_control_cui', anonymous=True)
    rospy.loginfo("init_node done")

    # Subscriber to /eyebrow_status
    rospy.Subscriber('/eyebrow_status', String, eyebrow_status_callback)
    rospy.loginfo("finished setting /eyebrow_status subsriber")
    
    rospy.spin()  # Keep the node running

if __name__ == '__main__':
    try:
        joint_control()
    except rospy.ROSInterruptException:
        pass
