#!/usr/bin/env python
import rospy
import random
import math
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

SDF_WRAPPER = """<?xml version='1.0'?><sdf version='1.6'><model name='pipeline_system'><static>true</static>{links}</model></sdf>"""
LINK_TMPL = """<link name='{name}'><pose>{x} {y} {z} {r} {p} {yw}</pose><visual name='v'><geometry><cylinder><radius>{rad}</radius><length>{len}</length></cylinder></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/{col}</name></script></material></visual><collision name='c'><geometry><cylinder><radius>{rad}</radius><length>{len}</length></cylinder></geometry></collision></link>"""

def get_pose(sx, sy, l, a):
    return sx + (l/2)*math.cos(a), sy + (l/2)*math.sin(a), 0.2, 0, 1.5708, a

def main():
    rospy.init_node('world_gen')
    links = ""
    
    total = random.uniform(5, 10)
    l1 = total * random.uniform(0.4, 0.6)
    l2 = total - l1
    a1 = random.uniform(0, 1.57)
    a2 = a1 + math.radians(random.uniform(-30, 30))

    cx, cy, cz, cr, cp, cyw = get_pose(1, 1, l1, a1)
    links += LINK_TMPL.format(name="s1", x=cx, y=cy, z=cz, r=cr, p=cp, yw=cyw, rad=0.1, len=l1, col="Yellow")
    
    ex1, ey1 = 1 + l1*math.cos(a1), 1 + l1*math.sin(a1)
    cx, cy, cz, cr, cp, cyw = get_pose(ex1, ey1, l2, a2)
    links += LINK_TMPL.format(name="s2", x=cx, y=cy, z=cz, r=cr, p=cp, yw=cyw, rad=0.1, len=l2, col="Yellow")

    taps = []
    for i in range(5):
        while True:
            d = random.uniform(0, total)
            if all(abs(d - t) > 0.75 for t in taps):
                taps.append(d)
                break
        
        if d <= l1:
            tx, ty, ta = 1 + d*math.cos(a1), 1 + d*math.sin(a1), a1 + 1.5708
        else:
            rem = d - l1
            tx, ty, ta = ex1 + rem*math.cos(a2), ey1 + rem*math.sin(a2), a2 + 1.5708
            
        fx, fy = tx + 0.25*math.cos(ta), ty + 0.25*math.sin(ta)
        links += LINK_TMPL.format(name="t{}".format(i), x=fx, y=fy, z=0.2, r=0, p=1.5708, yw=ta, rad=0.05, len=0.5, col="Red")

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)("pipeline_system", SDF_WRAPPER.format(links=links), "", Pose(), "world")

if __name__ == '__main__':
    main()