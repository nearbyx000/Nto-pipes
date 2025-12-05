#!/usr/bin/env python
import rospy
import cv2
import math
import json
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from clover import srv

class Node:
    def __init__(self):
        rospy.init_node('mission')
        self.br = CvBridge()
        self.nav = rospy.ServiceProxy('navigate', srv.Navigate)
        self.telem = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.land = rospy.ServiceProxy('land', Trigger)
        self.pub = rospy.Publisher('/tubes', String, queue_size=10)
        
        rospy.Subscriber('/mission_cmd', String, self.cmd_cb)
        rospy.Subscriber('main_camera/image_raw', Image, self.img_cb)
        
        self.run = False
        self.taps = []
        self.pos = [0, 0]

    def cmd_cb(self, msg):
        if msg.data == 'start': self.run = True; self.start_mission()
        elif msg.data == 'stop': self.run = False; self.land()
        elif msg.data == 'kill': self.run = False; self.nav(x=0, y=0, z=0, frame_id='body', auto_arm=False)

    def img_cb(self, data):
        if not self.run: return
        try:
            cv_img = self.br.imgmsg_to_cv2(data, 'bgr8')
            hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255)) + cv2.inRange(hsv, (170, 100, 100), (180, 255, 255))
            
            for cnt in cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]:
                if cv2.contourArea(cnt) > 300:
                    t = self.telem(frame_id='aruco_map')
                    if all(math.hypot(t.x-p[0], t.y-p[1]) > 1.0 for p in self.taps):
                        self.taps.append([round(t.x, 2), round(t.y, 2)])
            
            self.pub.publish(json.dumps({'drone': [round(self.telem(frame_id='aruco_map').x, 2), round(self.telem(frame_id='aruco_map').y, 2)], 'taps': self.taps}))
        except: pass

    def start_mission(self):
        self.nav(x=0, y=0, z=2, frame_id='body', auto_arm=True); rospy.sleep(5)
        self.nav(x=1, y=1, z=2, speed=1, frame_id='aruco_map'); rospy.sleep(4)
        
        x, dr = 1.0, 1
        while x < 11.0 and self.run:
            pts = [(x, 1.0), (x, 10.0)] if dr == 1 else [(x, 10.0), (x, 1.0)]
            for p in pts:
                self.nav(x=p[0], y=p[1], z=2, speed=1, frame_id='aruco_map')
                while self.run:
                    t = self.telem(frame_id='aruco_map')
                    if math.hypot(t.x-p[0], t.y-p[1]) < 0.3: break
                    rospy.sleep(0.1)
            x += 1.5; dr *= -1

        if self.run: self.nav(x=0, y=0, z=1.5, speed=1.5, frame_id='aruco_map'); rospy.sleep(7); self.land()

if __name__ == '__main__':
    Node()
    rospy.spin()