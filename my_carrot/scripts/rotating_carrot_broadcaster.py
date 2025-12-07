#!/usr/bin/env python3
import rospy
import tf
import math
from tf.transformations import quaternion_from_euler
from turtlesim.msg import Pose

class RotatingCarrotBroadcaster:
    def __init__(self):
        rospy.init_node('rotating_carrot_broadcaster')
        
        # Получаем параметры
        self.turtle_name = rospy.get_param('~turtle_name', 'turtle1')
        
        # Параметры вращения морковки
        self.rotation_radius = rospy.get_param('~rotation_radius', 1.5)  # радиус вращения (метры)
        self.rotation_speed = rospy.get_param('~rotation_speed', 1.0)    # скорость вращения (рад/сек)
        self.carrot_name = rospy.get_param('~carrot_name', 'rotating_carrot')
        
        # Создаем TF broadcaster
        self.br = tf.TransformBroadcaster()
        
        # Время начала работы для расчета фазы вращения
        self.start_time = rospy.Time.now().to_sec()
        
        # Подписываемся на топик с позицией черепахи
        pose_topic = '/%s/pose' % self.turtle_name
        rospy.Subscriber(pose_topic, Pose, self.pose_callback)
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("Запущен Rotating Carrot Broadcaster")
        rospy.loginfo("Черепаха: %s", self.turtle_name)
        rospy.loginfo("Морковка: %s", self.carrot_name)
        rospy.loginfo("Радиус вращения: %.2f м", self.rotation_radius)
        rospy.loginfo("Скорость вращения: %.2f рад/с", self.rotation_speed)
        rospy.loginfo("=" * 50)
    
    def pose_callback(self, msg):
        current_time = rospy.Time.now()
        
        # 1. Публикуем преобразование world -> turtle
        self.br.sendTransform(
            (msg.x, msg.y, 0.0),                    # позиция (x, y, z)
            quaternion_from_euler(0, 0, msg.theta), # ориентация (кватернион)
            current_time,                           # временная метка
            self.turtle_name,                       # дочерний фрейм
            "world"                                 # родительский фрейм
        )
        
        # 2. Вычисляем позицию вращающейся морковки
        # Морковка вращается по окружности вокруг черепахи
        time_diff = current_time.to_sec() - self.start_time
        rotation_angle = self.rotation_speed * time_diff
        
        # Координаты морковки в системе turtle (относительные)
        carrot_rel_x = self.rotation_radius * math.cos(rotation_angle)
        carrot_rel_y = self.rotation_radius * math.sin(rotation_angle)
        
        # 3. Публикуем преобразование turtle -> rotating_carrot
        self.br.sendTransform(
            (carrot_rel_x, carrot_rel_y, 0.0),  # относительная позиция
            (0.0, 0.0, 0.0, 1.0),               # без вращения (единичный кватернион)
            current_time,
            self.carrot_name,                   # имя морковки
            self.turtle_name                    # относительно черепахи
        )
        
        # 4. Дополнительно: публикуем world -> rotating_carrot для удобства
        # Преобразуем относительные координаты в абсолютные
        cos_theta = math.cos(msg.theta)
        sin_theta = math.sin(msg.theta)
        
        carrot_abs_x = msg.x + carrot_rel_x * cos_theta - carrot_rel_y * sin_theta
        carrot_abs_y = msg.y + carrot_rel_x * sin_theta + carrot_rel_y * cos_theta
        
        self.br.sendTransform(
            (carrot_abs_x, carrot_abs_y, 0.0),
            quaternion_from_euler(0, 0, msg.theta + rotation_angle),  # морковка тоже вращается
            current_time,
            self.carrot_name + "_abs",  # абсолютные координаты
            "world"
        )
        
        # Логируем информацию раз в 3 секунды
        rospy.loginfo_throttle(3.0,
            "Черепаха: (%.2f, %.2f), Морковка относительно: (%.2f, %.2f), Угол: %.2f рад",
            msg.x, msg.y, carrot_rel_x, carrot_rel_y, rotation_angle)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        broadcaster = RotatingCarrotBroadcaster()
        broadcaster.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Завершение работы Rotating Carrot Broadcaster")
