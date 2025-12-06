#!/usr/bin/env python3
import rospy

# Простой тестовый узел
rospy.init_node("test_polynomial", anonymous=True)
rospy.loginfo("Test polynomial node started successfully!")
rospy.sleep(3)
rospy.loginfo("Test completed.")
