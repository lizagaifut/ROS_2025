#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray, Int32

def main():
    rospy.init_node("summing_node", anonymous=False)
    
    pub = rospy.Publisher("final_sum", Int32, queue_size=10)
    
    def callback(msg):
        if len(msg.data) == 3:
            total = sum(msg.data)
            rospy.loginfo(f"Summing: {msg.data} = {total}")
            
            sum_msg = Int32()
            sum_msg.data = total
            pub.publish(sum_msg)
    
    rospy.Subscriber("polynominal_results", Int32MultiArray, callback)
    rospy.loginfo("Summing node ready!")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
