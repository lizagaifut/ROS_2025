#!/usr/bin/env python3
import rospy
import tf
import math
from geometry_msgs.msg import Twist
import turtlesim.srv

class ChaseRotatingCarrot:
    def __init__(self):
        rospy.init_node('chase_rotating_carrot')
        
        # Параметры
        self.chaser_name = rospy.get_param('~chaser_name', 'turtle2')
        self.carrot_name = rospy.get_param('~carrot_name', 'rotating_carrot')
        
        # Коэффициенты П-регулятора
        self.k_linear = rospy.get_param('~k_linear', 1.5)
        self.k_angular = rospy.get_param('~k_angular', 4.0)
        
        # Максимальные скорости
        self.max_linear = rospy.get_param('~max_linear', 2.0)
        self.max_angular = rospy.get_param('~max_angular', 2.0)
        
        rospy.loginfo("Инициализация Chase Rotating Carrot...")
        
        # Даем время для запуска системы
        rospy.sleep(2.0)
        
        # Создаем черепаху-преследователя если ее нет
        self.create_chaser_turtle()
        
        # Создаем TF listener
        self.listener = tf.TransformListener()
        
        # Создаем publisher для управления преследователем
        cmd_topic = '/%s/cmd_vel' % self.chaser_name
        self.cmd_pub = rospy.Publisher(cmd_topic, Twist, queue_size=10)
        
        # Даем время для установления TF преобразований
        rospy.sleep(3.0)
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("Chase Rotating Carrot запущен!")
        rospy.loginfo("Преследователь: %s", self.chaser_name)
        rospy.loginfo("Цель: %s", self.carrot_name)
        rospy.loginfo("Коэффициенты: линейный=%.1f, угловой=%.1f", 
                     self.k_linear, self.k_angular)
        rospy.loginfo("=" * 50)
    
    def create_chaser_turtle(self):
        """Создает черепаху-преследователя если ее нет"""
        try:
            rospy.wait_for_service('/spawn', timeout=5.0)
            spawn = rospy.ServiceProxy('/spawn', turtlesim.srv.Spawn)
            
            # Позиция для новой черепахи
            spawn_x = rospy.get_param('~spawn_x', 3.0)
            spawn_y = rospy.get_param('~spawn_y', 3.0)
            
            response = spawn(spawn_x, spawn_y, 0.0, self.chaser_name)
            rospy.loginfo("Создана черепаха-преследователь: %s", response.name)
        except rospy.ServiceException as e:
            rospy.logwarn("Не удалось создать черепаху: %s", e)
            rospy.logwarn("Возможно, черепаха %s уже существует", self.chaser_name)
        except rospy.ROSException:
            rospy.logerr("Сервис /spawn недоступен!")
    
    def run(self):
        rate = rospy.Rate(10.0)  # 10 Гц
        
        error_count = 0
        max_errors = 20
        
        while not rospy.is_shutdown() and error_count < max_errors:
            try:
                # Получаем преобразование от преследователя к морковке
                (trans, rot) = self.listener.lookupTransform(
                    self.chaser_name, 
                    self.carrot_name, 
                    rospy.Time(0)
                )
                
                # Вычисляем расстояние до цели
                distance = math.sqrt(trans[0]**2 + trans[1]**2)
                
                # Вычисляем угол до цели (направление)
                angle_to_target = math.atan2(trans[1], trans[0])
                
                # Создаем команду управления (П-регулятор)
                cmd = Twist()
                cmd.linear.x = self.k_linear * distance
                cmd.angular.z = self.k_angular * angle_to_target
                
                # Ограничиваем скорость
                cmd.linear.x = min(max(cmd.linear.x, 0.0), self.max_linear)
                cmd.angular.z = max(min(cmd.angular.z, self.max_angular), -self.max_angular)
                
                # Публикуем команду
                self.cmd_pub.publish(cmd)
                
                # Логируем информацию раз в секунду
                rospy.loginfo_throttle(1.0,
                    "До морковки: расстояние=%.2f м, угол=%.2f рад, скорость=%.2f м/с",
                    distance, angle_to_target, cmd.linear.x)
                
                error_count = 0  # сбрасываем счетчик ошибок
                
            except tf.LookupException:
                rospy.logwarn_throttle(2.0, 
                    "Не найдено преобразование %s -> %s", 
                    self.chaser_name, self.carrot_name)
                self.stop_turtle()
                error_count += 1
                
            except tf.ConnectivityException:
                rospy.logwarn_throttle(2.0, "Нет связи между фреймами TF")
                self.stop_turtle()
                error_count += 1
                
            except tf.ExtrapolationException:
                rospy.logwarn_throttle(2.0, "Проблема с временными метками TF")
                self.stop_turtle()
                error_count += 1
                
            except Exception as e:
                rospy.logerr("Неожиданная ошибка: %s", e)
                self.stop_turtle()
                error_count += 1
            
            rate.sleep()
        
        if error_count >= max_errors:
            rospy.logerr("Слишком много ошибок, остановка преследования")
            self.stop_turtle()
    
    def stop_turtle(self):
        """Останавливает черепаху"""
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)

if __name__ == '__main__':
    try:
        chaser = ChaseRotatingCarrot()
        chaser.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Завершение работы Chase Rotating Carrot")
