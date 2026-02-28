#!/usr/bin/env python

import rospy
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from nav_msgs.srv import GetPlan
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion
from actionlib_msgs.msg import GoalID
import math

class FuzzyNavigator:
    def __init__(self):
        rospy.init_node('fuzzy_navigator', anonymous=False)

        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_nav', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)

        self.robot_pose = None
        self.robot_yaw = None
        self.laser_scan_data = None
        
        self.state = 'IDLE' # Can be IDLE or NAVIGATING
        self.global_path = []
        self.current_waypoint_index = 0
        
        self.LINEAR_SPEED = 0.3         # Increased: Turtlebot3 max speed
        self.GOAL_RADIUS = 0.3          # Increased: How close we need to get to a waypoint

        # Wait for muxer service to be available
        try:
            rospy.wait_for_service('/cmd_vel_mux/select', timeout=5.0)
            self.mux_select = rospy.ServiceProxy('/cmd_vel_mux/select', Empty)
            rospy.loginfo("Velocity muxer service found")
        except rospy.ROSException:
            rospy.logwarn("Velocity muxer service not found - robot may not move!")
            self.mux_select = None

        # --- FUZZY LOGIC SETUP ---
        self.setup_fuzzy_controller()
        rospy.loginfo("Fuzzy logic controller set up.")
        rospy.loginfo("Fuzzy Navigator initialized. Waiting for goal from RViz...")

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        (_, _, self.robot_yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def scan_callback(self, msg):
        self.laser_scan_data = msg

    def setup_fuzzy_controller(self):
        # Antecedents (Inputs)
        distance = ctrl.Antecedent(np.arange(0, 3.5, 0.1), 'distance')
        angle = ctrl.Antecedent(np.arange(-np.pi, np.pi, 0.1), 'angle')
        
        # Consequent (Output)
        steering = ctrl.Consequent(np.arange(-1.5, 1.6, 0.1), 'steering')

        # Membership Functions for Distance
        distance['near'] = fuzz.trapmf(distance.universe, [0, 0, 0.2, 0.5])
        distance['medium'] = fuzz.trimf(distance.universe, [0.3, 0.7, 1.2])
        distance['far'] = fuzz.trapmf(distance.universe, [1, 2, 3.5, 3.5])

        # Membership Functions for Goal Angle
        angle['left'] = fuzz.trimf(angle.universe, [-np.pi, -np.pi/2, 0])
        angle['straight'] = fuzz.trimf(angle.universe, [-np.pi/4, 0, np.pi/4])
        angle['right'] = fuzz.trimf(angle.universe, [0, np.pi/2, np.pi])

        # Membership Functions for Steering
        steering['sharp_left'] = fuzz.trimf(steering.universe, [-1.5, -1.2, -0.8])
        steering['left'] = fuzz.trimf(steering.universe, [-1.0, -0.5, 0])
        steering['straight'] = fuzz.trimf(steering.universe, [-0.2, 0, 0.2])
        steering['right'] = fuzz.trimf(steering.universe, [0, 0.5, 1.0])
        steering['sharp_right'] = fuzz.trimf(steering.universe, [0.8, 1.2, 1.5])

        # Fuzzy Rules
        rule1 = ctrl.Rule(distance['near'], steering['sharp_right'])
        rule2 = ctrl.Rule(distance['medium'] & angle['left'], steering['right'])
        rule3 = ctrl.Rule(distance['medium'] & angle['right'], steering['left'])
        rule4 = ctrl.Rule(distance['far'] & angle['left'], steering['left'])
        rule5 = ctrl.Rule(distance['far'] & angle['right'], steering['right'])
        rule6 = ctrl.Rule(distance['far'] & angle['straight'], steering['straight'])
        rule7 = ctrl.Rule(distance['medium'] & angle['straight'], steering['right']) # Veer right if medium obstacle is ahead

        self.steering_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7])
        self.steering_sim = ctrl.ControlSystemSimulation(self.steering_ctrl)

    def fuzzy_navigate(self):
        if self.laser_scan_data is None or self.robot_pose is None or not self.global_path:
            rospy.logwarn("Cannot navigate - missing data")
            self.stop_robot()
            return
            
        # Get Inputs for Fuzzy Controller
        # 1. Min distance in a 90-degree frontal arc
        ranges = self.laser_scan_data.ranges
        front_ranges = ranges[-45:] + ranges[:45]
        min_dist_front = min([r for r in front_ranges if r > 0.01])

        # 2. Angle to next waypoint
        waypoint = self.global_path[self.current_waypoint_index].pose.position
        angle_to_waypoint = math.atan2(waypoint.y - self.robot_pose.y, waypoint.x - self.robot_pose.x)
        angle_diff = self.normalize_angle(angle_to_waypoint - self.robot_yaw)

        # Feed inputs to Fuzzy System
        self.steering_sim.input['distance'] = min_dist_front
        self.steering_sim.input['angle'] = angle_diff
        self.steering_sim.compute()
        steering_output = self.steering_sim.output['steering']
        
        # Publish Commands
        twist_msg = Twist()
        # Reduce speed if turning sharply or if an obstacle is very close
        if abs(steering_output) > 0.8 or min_dist_front < 0.4:
            twist_msg.linear.x = self.LINEAR_SPEED * 0.5
        else:
            twist_msg.linear.x = self.LINEAR_SPEED
        
        # ROS standard: positive angular.z is a counter-clockwise (left) turn
        # Our fuzzy system outputs positive for right turn, so we negate it.
        twist_msg.angular.z = steering_output * -1.0 
        
        self.cmd_vel_pub.publish(twist_msg)
        rospy.loginfo(f"Fuzzy navigation: linear={twist_msg.linear.x:.2f}, angular={twist_msg.angular.z:.2f}, front_dist={min_dist_front:.2f}, angle_diff={angle_diff:.2f}")

        # Check for waypoint reached
        distance_to_waypoint = math.sqrt((waypoint.x - self.robot_pose.x)**2 + (waypoint.y - self.robot_pose.y)**2)
        if distance_to_waypoint < self.GOAL_RADIUS:
            rospy.loginfo(f"Waypoint {self.current_waypoint_index} reached.")
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.global_path):
                rospy.loginfo("Final goal reached!")
                self.state = 'IDLE'
                self.stop_robot()

    def rviz_goal_callback(self, msg):
        rospy.loginfo("Received a goal from RViz! Requesting path...")
        if self.robot_pose is None:
            rospy.logwarn("Cannot request plan, robot pose is not yet available.")
            return
        
        # Switch muxer to our navigator
        if self.mux_select:
            try:
                self.mux_select()
                rospy.loginfo("Switched velocity control to fuzzy navigator")
            except rospy.ServiceException as e:
                rospy.logwarn(f"Failed to switch muxer: {e}")
        else:
            rospy.logwarn("No muxer available - robot may not move!")
        
        # Cancel any active move_base goals first
        try:
            cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
            rospy.sleep(0.1)  # Give publisher time to connect
            cancel_msg = GoalID()
            cancel_pub.publish(cancel_msg)
            rospy.sleep(0.5)  # Wait for move_base to become inactive
            rospy.loginfo("Cancelled any active move_base goals.")
        except Exception as e:
            rospy.logwarn(f"Failed to cancel goals: {e}")
        
        # Get the path directly by calling the service
        make_plan_service = rospy.ServiceProxy('/move_base/NavfnROS/make_plan', GetPlan)
        start = PoseStamped()
        start.header.frame_id = 'map'
        start.header.stamp = rospy.Time(0)  # Use latest available transform
        start.pose.position = self.robot_pose
        start.pose.orientation = self.get_current_orientation()

        goal = msg
        goal.header.stamp = rospy.Time(0)  # Use latest available transform
        
        try:
            plan = make_plan_service(start=start, goal=goal, tolerance=0.5)
            if plan.plan.poses:
                # Sample waypoints to reduce density
                original_count = len(plan.plan.poses)
                self.global_path = self.sample_waypoints(plan.plan.poses, min_distance=0.15)
                self.current_waypoint_index = 0
                self.state = 'NAVIGATING'
                rospy.loginfo(f"Path received with {original_count} waypoints, sampled down to {len(self.global_path)} waypoints. Starting fuzzy navigation.")
            else:
                rospy.logwarn("Move_base failed to generate a plan.")
                self.state = 'IDLE'
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to make_plan failed: {e}")
            self.state = 'IDLE'

    def sample_waypoints(self, waypoints, min_distance=0.15):
        """Sample waypoints to reduce path density and create more stable navigation."""
        if not waypoints:
            return waypoints
            
        sampled = [waypoints[0]]  # Always keep the first waypoint
        
        for i in range(1, len(waypoints)):
            current_pos = waypoints[i].pose.position
            last_sampled_pos = sampled[-1].pose.position
            
            # Calculate distance from last sampled waypoint
            distance = math.sqrt(
                (current_pos.x - last_sampled_pos.x)**2 + 
                (current_pos.y - last_sampled_pos.y)**2
            )
            
            # Only add waypoint if it's far enough from the last sampled one
            if distance >= min_distance:
                sampled.append(waypoints[i])
        
        # Always keep the final waypoint (goal)
        if sampled[-1] != waypoints[-1]:
            sampled.append(waypoints[-1])
            
        return sampled

    def get_current_orientation(self):
        # Helper to get orientation message
        odom_msg = rospy.wait_for_message('/odom', Odometry)
        return odom_msg.pose.pose.orientation

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.state == 'NAVIGATING':
                self.fuzzy_navigate()
            else:
                self.stop_robot()
            rate.sleep()

if __name__ == '__main__':
    try:
        navigator = FuzzyNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Fuzzy navigator node shut down.")