#!/usr/bin/env python3
import rospy
import sys
from std_msgs.msg import Int32MultiArray, Int32

def main():
    if len(sys.argv) != 4:
        print("Usage: request_node.py num1 num2 num3")
        return
        
    nums = [int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3])]
    
    # Уникальное анонимное имя
    rospy.init_node("request_node", anonymous=True)
    
    result = {"value": None}
    
    def callback(msg):
        result["value"] = msg.data
        rospy.signal_shutdown("Got result")
    
    rospy.Subscriber("final_sum", Int32, callback)
    
    # Ждем подключения
    pub = rospy.Publisher("input_numbers", Int32MultiArray, queue_size=10)
    rospy.sleep(1)
    
    # Отправляем запрос
    msg = Int32MultiArray()
    msg.data = nums
    pub.publish(msg)
    print(f"Sent: {nums}")
    
    # Ждем результат
    timeout = 5  # секунд
    start_time = rospy.Time.now()
    
    while (result["value"] is None and 
           (rospy.Time.now() - start_time).to_sec() < timeout):
        rospy.sleep(0.1)
    
    if result["value"] is not None:
        print(f"Result: {result['value']}")
    else:
        print("Timeout: No result received")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
