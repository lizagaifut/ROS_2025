#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray

def main():
    rospy.init_node("polynominal_node", anonymous=False)
    
    pub = rospy.Publisher("polynominal_results", Int32MultiArray, queue_size=10)
    
    def callback(msg):
        if len(msg.data) == 3:
            a, b, c = msg.data
            results = [a**2, b**3, c**4]
            rospy.loginfo(f"Input: {msg.data}, Output: {results}")
            
            result_msg = Int32MultiArray()
            result_msg.data = results
            pub.publish(result_msg)
    
    rospy.Subscriber("input_numbers", Int32MultiArray, callback)
    rospy.loginfo("Polynominal node ready!")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
