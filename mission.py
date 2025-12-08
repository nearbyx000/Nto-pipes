import rospy
import math
import json
import threading
from std_msgs.msg import String
from clover import srv
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped # Import PoseStamped

class MissionControl:
    def __init__(self):
        print("STARTING MISSION NODE")
        rospy.init_node('mission_control')
        
        # --- Services ---
        rospy.loginfo("Waiting for /navigate service...")
        rospy.wait_for_service('navigate')
        self.nav = rospy.ServiceProxy('navigate', srv.Navigate)
        rospy.loginfo("/navigate service available.")

        # Do not wait for get_telemetry service, we will subscribe directly
        rospy.loginfo("Waiting for /land service...")
        rospy.wait_for_service('land')
        self.land = rospy.ServiceProxy('land', Trigger)
        rospy.loginfo("/land service available.")
        
        # --- Pub/Sub ---
        self.state_pub = rospy.Publisher('/tubes', String, queue_size=10)
        rospy.Subscriber('/mission_cmd', String, self.handle_command)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_cb) # Subscribe to MAVROS pose
        
        # --- State ---
        self.running = False
        self.mission_thread = None
        self.ground_truth_taps = []
        self.detected_taps = [] # List of [x, y]
        self.current_pose = None # Stores latest PoseStamped message
        
        # Load Ground Truth from Params (set by world-gen.py)
        for _ in range(10):
            if rospy.has_param('/pipeline/taps'):
                self.ground_truth_taps = rospy.get_param('/pipeline/taps')
                break
            rospy.sleep(1)
            
        rospy.loginfo("Mission Node Ready. Ground Truth Taps: {}".format(len(self.ground_truth_taps)))

    def _pose_cb(self, msg):
        """Callback for /mavros/local_position/pose topic"""
        self.current_pose = msg
        
    def handle_command(self, msg):
        cmd = msg.data
        rospy.loginfo("Received command: {}".format(cmd))
        
        if cmd == 'start':
            if not self.running:
                self.running = True
                self.mission_thread = threading.Thread(target=self.run_mission_logic)
                self.mission_thread.start()
        
        elif cmd == 'stop':
            rospy.loginfo("Received command: stop. Setting self.running to False.")
            self.running = False
            self.land()
            
        elif cmd == 'kill':
            rospy.loginfo("Received command: kill. Setting self.running to False.")
            self.running = False
            # Emergency drop: disarm immediately
            self.nav(x=0, y=0, z=0, frame_id='body', auto_arm=False)

    def publish_state(self):
        """Publishes current drone pos and list of found taps to Web UI"""
        if self.current_pose is None:
            rospy.logwarn("Telemetry not yet available for publishing.")
            return

        drone_x = self.current_pose.pose.position.x
        drone_y = self.current_pose.pose.position.y
        
        state = {
            'drone': [round(drone_x, 2), round(drone_y, 2)],
            'taps': self.detected_taps
        }
        self.state_pub.publish(json.dumps(state))

    def check_for_taps(self):
        """Simulates camera detection based on proximity"""
        if self.current_pose is None:
            return

        drone_x = self.current_pose.pose.position.x
        drone_y = self.current_pose.pose.position.y
        
        for gt_tap in self.ground_truth_taps:
            dist = math.hypot(drone_x - gt_tap[0], drone_y - gt_tap[1])
            
            # If within 1.0m, consider it "seen"
            if dist < 1.0:
                # Check if we already recorded this one (avoid duplicates)
                already_found = False
                for dt in self.detected_taps:
                    if math.hypot(dt[0] - gt_tap[0], dt[1] - gt_tap[1]) < 0.5:
                        already_found = True
                        break
                
                if not already_found:
                    rospy.loginfo("Visual Detection! Tap at [{:.2f}, {:.2f}]".format(gt_tap[0], gt_tap[1]))
                    self.detected_taps.append(gt_tap)

    def navigate_and_scan(self, x, y, z=2.0, speed=1.0, frame_id='map', tolerance=0.3):
        """Blocking navigation that updates state while flying"""
        if not self.running: return

        self.nav(x=x, y=y, z=z, speed=speed, frame_id=frame_id)
        
        while self.running:
            rospy.loginfo("DEBUG: navigate_and_scan loop checking self.running flag.")
            if self.current_pose is None:
                rospy.logwarn("Waiting for telemetry for navigation check...")
                rospy.sleep(0.5)
                continue

            drone_x = self.current_pose.pose.position.x
            drone_y = self.current_pose.pose.position.y
            
            # 1. Update Detection
            self.check_for_taps()
            
            # 2. Publish State
            self.publish_state()
            
            # 3. Check arrival
            dist_to_target = math.hypot(drone_x - x, drone_y - y)
            if dist_to_target < tolerance:
                break
                
            rospy.sleep(0.2)

    def run_mission_logic(self):
        rospy.loginfo("Mission Started. Taking off...")
        
        # Arm and Takeoff
        self.nav(x=0, y=0, z=2, frame_id='body', auto_arm=True)
        rospy.sleep(5)
        
        # Define Scan Pattern (Simple Zig-Zag)
        # Area: X from 0 to 12, Y from 0 to 12
        waypoints = [
            [1, 1], [1, 10], 
            [3, 10], [3, 1],
            [5, 1], [5, 10],
            [7, 10], [7, 1],
            [9, 1], [9, 10],
            [11, 10], [11, 1]
        ]
        
        for wp in waypoints:
            if not self.running: break
            rospy.loginfo("Navigating to waypoint: {}".format(wp))
            self.navigate_and_scan(wp[0], wp[1])
            
        rospy.loginfo("Scan Complete. Returning to Base.")
        self.navigate_and_scan(0, 0)
        self.land()
        self.running = False

if __name__ == '__main__':
    MissionControl()
    rospy.spin()
