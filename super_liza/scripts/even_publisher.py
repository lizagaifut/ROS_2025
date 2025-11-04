#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32, String  # Добавили String в импорт

def even_publisher():
    rospy.init_node('even_publisher', anonymous=True)
    pub = rospy.Publisher('even_numbers', Int32, queue_size=10)
    overflow_pub = rospy.Publisher('overflow_signal', String, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz
    counter = 0
    
    while not rospy.is_shutdown():
        # Публикуем четное число
        pub.publish(counter)
        rospy.loginfo("Published even number: %d", counter)
        
        # Проверяем переполнение
        if counter >= 100:
            overflow_msg = "Counter overflow at value: %d" % counter
            overflow_pub.publish(overflow_msg)
            rospy.logwarn(overflow_msg)
            counter = 0  # Сбрасываем счетчик
        else:
            counter += 2  # Увеличиваем на 2 для след. четного числа
        
        rate.sleep()

if __name__ == '__main__':
    try:
        even_publisher()
    except rospy.ROSInterruptException:
        pass
