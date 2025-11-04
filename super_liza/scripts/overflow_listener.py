#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def overflow_callback(msg):
    rospy.loginfo("OVERFLOW DETECTED: %s", msg.data)

def overflow_listener():
    rospy.init_node('overflow_listener', anonymous=True)
    rospy.Subscriber('overflow_signal', String, overflow_callback, queue_size=10)
    
    rospy.loginfo("Overflow listener started. Waiting for overflow signals...")
    rospy.spin()

if __name__ == '__main__':
    try:
        overflow_listener()
    except rospy.ROSInterruptException:
        pass
