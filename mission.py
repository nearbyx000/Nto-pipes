import rospy
import cv2
import math
import json
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from clover import srv

class MissionNode:
    def __init__(self):
        rospy.init_node('oil_mission')
        
        self.bridge = CvBridge()
        self.telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.land = rospy.ServiceProxy('land', srv.Trigger)
        
        self.pub_tubes = rospy.Publisher('/tubes', String, queue_size=10)
        self.sub_cmd = rospy.Subscriber('/mission_cmd', String, self.cmd_callback)
        self.sub_cam = rospy.Subscriber('main_camera/image_raw', Image, self.img_callback)
        
        self.running = False
        self.detected_taps = []
        self.current_pos = [0, 0]
        
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])

    def cmd_callback(self, msg):
        if msg.data == 'start' and not self.running:
            self.running = True
            self.run_mission()
        elif msg.data == 'stop':
            self.running = False
            self.land()
        elif msg.data == 'kill':
            self.running = False
            rospy.ServiceProxy('navigate', srv.Navigate)(x=0, y=0, z=0, frame_id='body', auto_arm=False)

    def img_callback(self, data):
        if not self.running:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
            mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
            mask = mask1 + mask2
            
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 500:
                    telem = self.telemetry(frame_id='aruco_map')
                    x, y = telem.x, telem.y
                    
                    is_new = True
                    for tap in self.detected_taps:
                        dist = math.sqrt((tap[0]-x)**2 + (tap[1]-y)**2)
                        if dist < 1.0:
                            is_new = False
                            break
                    
                    if is_new:
                        self.detected_taps.append([round(x, 2), round(y, 2)])
                        
            msg_data = {
                'drone': [round(self.current_pos[0], 2), round(self.current_pos[1], 2)],
                'taps': self.detected_taps
            }
            self.pub_tubes.publish(json.dumps(msg_data))
            
        except Exception:
            pass

    def run_mission(self):
        # 1. Взлет
        self.navigate(x=0, y=0, z=1.5, frame_id='body', auto_arm=True)
        rospy.sleep(5)
        
        # 2. Выход на начало зоны поиска (Точка 1,1 - начало трубы)
        # Поднимаемся чуть выше (2м), чтобы захватить больше области камерой
        self.navigate(x=1, y=1, z=2.0, speed=1, frame_id='aruco_map')
        rospy.sleep(4)
        
        # 3. Генерация точек для "Змейки" (Сканирование области)
        # Мы предполагаем, что труба длиной до 10м находится в секторе X+ Y+
        waypoints = []
        
        # Сканируем область 8x8 метров с шагом 2 метра
        # Это гарантирует, что мы пересечем трубу, как бы она ни легла
        scan_width = 10 
        step = 1.5 # Шаг между проходами (зависит от высоты и угла обзора камеры)
        
        current_x = 1.0
        direction = 1 # 1 = вверх, -1 = вниз
        
        while current_x < 11.0: # Летим до X=11
            # Точка начала полосы
            y_start = 1.0 if direction == 1 else 10.0
            # Точка конца полосы
            y_end = 10.0 if direction == 1 else 1.0
            
            waypoints.append((current_x, y_start))
            waypoints.append((current_x, y_end))
            
            current_x += step
            direction *= -1 # Меняем направление для следующей полосы

        # 4. Выполнение полета по точкам
        for p in waypoints:
            if not self.running: break
            
            # Летим к точке
            self.navigate(x=p[0], y=p[1], z=2.0, speed=1.0, frame_id='aruco_map')
            
            # Ждем пока долетит (простая проверка расстояния)
            while self.running:
                telem = self.telemetry(frame_id='aruco_map')
                dist = math.sqrt((telem.x - p[0])**2 + (telem.y - p[1])**2)
                self.current_pos = [telem.x, telem.y] # Обновляем позицию для веба
                
                # Если подлетели ближе 0.3м - следующая точка
                if dist < 0.3:
                    break
                rospy.sleep(0.2)

        # 5. Возврат домой
        if self.running:
            self.navigate(x=0, y=0, z=1.5, speed=1.5, frame_id='aruco_map')
            rospy.sleep(7)
            self.land()
            self.running = False
if __name__ == '__main__':
    node = MissionNode()
    rospy.spin()