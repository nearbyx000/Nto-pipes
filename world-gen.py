#!/usr/bin/env python
import rospy
import random
import math
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

# Constants
PI = 3.14159265359
PIPE_RADIUS = 0.1
TAP_RADIUS = 0.05
TAP_LENGTH = 0.5

SDF_WRAPPER = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='pipeline_system'>
    <static>true</static>
    {links}
  </model>
</sdf>"""

LINK_TMPL = """
    <link name='{name}'>
      <pose>{x} {y} {z} {r} {p} {yw}</pose>
      <visual name='visual'>
        <geometry>
          <cylinder>
            <radius>{rad}</radius>
            <length>{len}</length>
          </cylinder>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/{col}</name>
          </script>
        </material>
      </visual>
      <collision name='collision'>
        <geometry>
          <cylinder>
            <radius>{rad}</radius>
            <length>{len}</length>
          </cylinder>
        </geometry>
      </collision>
    </link>"""

def get_link_pose(start_x, start_y, length, angle):
    """Calculates the center pose (x, y) for a cylinder link."""
    center_x = start_x + (length / 2.0) * math.cos(angle)
    center_y = start_y + (length / 2.0) * math.sin(angle)
    return center_x, center_y

def generate_world():
    rospy.init_node('world_generator')
    
    links_xml = ""
    
    # --- 1. Generate Main Pipeline (2 segments with a bend) ---
    total_len = random.uniform(5, 10)
    len1 = total_len * random.uniform(0.4, 0.6)
    len2 = total_len - len1
    
    angle1 = random.uniform(0, PI/2) # 0 to 90 deg
    angle2 = angle1 + math.radians(random.uniform(-30, 30)) # Bend

    # Segment 1
    cx1, cy1 = get_link_pose(1, 1, len1, angle1)
    links_xml += LINK_TMPL.format(name="pipe1", x=cx1, y=cy1, z=0.2, r=0, p=PI/2, yw=angle1, rad=PIPE_RADIUS, len=len1, col="Yellow")
    
    # Elbow Joint
    elbow_x = 1 + len1 * math.cos(angle1)
    elbow_y = 1 + len1 * math.sin(angle1)
    
    # Segment 2
    cx2, cy2 = get_link_pose(elbow_x, elbow_y, len2, angle2)
    links_xml += LINK_TMPL.format(name="pipe2", x=cx2, y=cy2, z=0.2, r=0, p=PI/2, yw=angle2, rad=PIPE_RADIUS, len=len2, col="Yellow")

    # --- 2. Generate Taps ---
    tap_locations = []
    taps_xml = ""
    
    attempts = 0
    while len(tap_locations) < 5 and attempts < 100:
        attempts += 1
        dist = random.uniform(0, total_len)
        
        # Minimum distance check (0.75m)
        if any(abs(dist - t['dist']) < 0.75 for t in tap_locations):
            continue
            
        # Calculate 3D position
        if dist <= len1:
            base_x = 1 + dist * math.cos(angle1)
            base_y = 1 + dist * math.sin(angle1)
            tap_angle = angle1 + PI/2
        else:
            rem = dist - len1
            base_x = elbow_x + rem * math.cos(angle2)
            base_y = elbow_y + rem * math.sin(angle2)
            tap_angle = angle2 + PI/2

        # Offset tap slightly from pipe center
        tx = base_x + 0.25 * math.cos(tap_angle)
        ty = base_y + 0.25 * math.sin(tap_angle)
        
        tap_locations.append({'dist': dist, 'x': tx, 'y': ty})
        
        # Add tap visual
        links_xml += LINK_TMPL.format(name="tap_{}".format(len(tap_locations)), x=tx, y=ty, z=0.2, r=0, p=PI/2, yw=tap_angle, rad=TAP_RADIUS, len=TAP_LENGTH, col="Red")

    # --- 3. Publish Ground Truth to ROS Params ---
    # Using ROS Params is safer than topics for static map data
    rospy.set_param('/pipeline/taps', [[t['x'], t['y']] for t in tap_locations])
    rospy.loginfo("Set /pipeline/taps param with {} taps".format(len(tap_locations)))

    # --- 4. Spawn in Gazebo ---
    rospy.loginfo("Spawning pipeline model...")
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn("pipeline_system", SDF_WRAPPER.format(links=links_xml), "", Pose(), "world")
        rospy.loginfo("Pipeline spawned successfully.")
    except rospy.ServiceException as e:
        rospy.logerr("Spawn failed: {}".format(e))

if __name__ == '__main__':
    try:
        generate_world()
    except rospy.ROSInterruptException:
        pass