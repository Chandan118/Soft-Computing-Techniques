#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from nav_msgs.srv import GetPlan
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion
from actionlib_msgs.msg import GoalID
import math

class HybridNavigatorDirect:
    def __init__(self):
        rospy.init_node('hybrid_navigator_direct', anonymous=False)

        # Publishers and Subscribers - DIRECT TO CMD_VEL
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        # We subscribe to the goal topic published by RViz
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)

        # Robot state variables
        self.robot_pose = None
        self.robot_yaw = None
        self.laser_scan_data = None
        
        # Navigation state machine
        self.state = 'IDLE' # Can be IDLE, NAVIGATING, OBSTACLE_AVOIDANCE
        self.global_path = []
        self.current_waypoint_index = 0
        
        # --- TUNABLE PARAMETERS ---
        self.GOAL_RADIUS = 0.05  # meters - reduced from 0.2 to prevent immediate waypoint completion
        self.OBSTACLE_THRESHOLD = 0.4   # How close an obstacle can be before we react
        self.LINEAR_SPEED = 0.2         # m/s
        self.ANGULAR_SPEED = 0.5        # rad/s
        self.WALL_FOLLOW_DISTANCE = 0.3 # Desired distance to keep from a wall

        rospy.loginfo("Hybrid Navigator (Direct) initialized. Publishing directly to /cmd_vel")
        rospy.loginfo("Waiting for goal from RViz...")

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        (_, _, self.robot_yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def scan_callback(self, msg):
        self.laser_scan_data = msg

    def rviz_goal_callback(self, msg):
        rospy.loginfo("Received a goal from RViz! Requesting path...")
        if self.robot_pose is None:
            rospy.logwarn("Cannot request plan, robot pose is not yet available.")
            return

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
                rospy.loginfo(f"Path received with {original_count} waypoints, sampled down to {len(self.global_path)} waypoints. Starting navigation.")
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

    def is_obstacle_in_front(self):
        if self.laser_scan_data is None: return False
        
        # Check a 60-degree arc in front of the robot
        ranges = self.laser_scan_data.ranges
        front_ranges = ranges[-30:] + ranges[:30]
        min_distance = min([r for r in front_ranges if r > 0.01])
        
        if min_distance < self.OBSTACLE_THRESHOLD:
            rospy.logwarn(f"Obstacle detected at {min_distance:.2f}m. Switching to OBSTACLE_AVOIDANCE.")
            return True
        return False

    def follow_global_path(self):
        if not self.global_path or self.robot_pose is None or self.robot_yaw is None: 
            rospy.logwarn("Cannot follow path - missing data")
            return

        # Get current waypoint
        waypoint = self.global_path[self.current_waypoint_index].pose.position
        
        # Check if waypoint is reached
        distance_to_waypoint = math.sqrt((waypoint.x - self.robot_pose.x)**2 + (waypoint.y - self.robot_pose.y)**2)
        
        # DEBUG: Print position info
        rospy.loginfo_throttle(1, f"Robot: ({self.robot_pose.x:.3f}, {self.robot_pose.y:.3f}), "
                                  f"Waypoint {self.current_waypoint_index}: ({waypoint.x:.3f}, {waypoint.y:.3f}), "
                                  f"Distance: {distance_to_waypoint:.3f}m")
        
        if distance_to_waypoint < self.GOAL_RADIUS:
            rospy.loginfo(f"Waypoint {self.current_waypoint_index} reached.")
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.global_path):
                rospy.loginfo("Final goal reached!")
                self.state = 'IDLE'
                self.stop_robot()
                return

        # Calculate heading to waypoint
        angle_to_waypoint = math.atan2(waypoint.y - self.robot_pose.y, waypoint.x - self.robot_pose.x)
        angle_diff = angle_to_waypoint - self.robot_yaw
        
        # Normalize angle to be between -pi and pi
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        if angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Proportional control
        twist_msg = Twist()
        if abs(angle_diff) > 0.3:  # 0.3 radians (~17 degrees)
            twist_msg.angular.z = self.ANGULAR_SPEED * (1 if angle_diff > 0 else -1)
        else:
            twist_msg.linear.x = self.LINEAR_SPEED
            twist_msg.angular.z = 0
            
        # DEBUG: Print what we're commanding
        rospy.loginfo_throttle(2, f"Commanding: linear={twist_msg.linear.x:.3f}, angular={twist_msg.angular.z:.3f}")
        
        self.cmd_vel_pub.publish(twist_msg)

    def avoid_obstacle_bug(self):
        """A simple Bug-like algorithm: turn and follow the wall."""
        if self.laser_scan_data is None: return

        ranges = self.laser_scan_data.ranges
        twist_msg = Twist()
        # Basic wall following: try to keep obstacle on the left
        # Get ranges for front and left-front
        front_dist = ranges[0]
        left_front_dist = ranges[45]

        # If too close in front, turn right
        if front_dist < self.WALL_FOLLOW_DISTANCE:
            twist_msg.linear.x = 0.05
            twist_msg.angular.z = -self.ANGULAR_SPEED
        # If space on the left, turn towards it
        elif left_front_dist > self.WALL_FOLLOW_DISTANCE * 1.2:
            twist_msg.linear.x = self.LINEAR_SPEED
            twist_msg.angular.z = self.ANGULAR_SPEED * 0.5
        # If too close on the left, turn away
        elif left_front_dist < self.WALL_FOLLOW_DISTANCE * 0.8:
            twist_msg.linear.x = self.LINEAR_SPEED
            twist_msg.angular.z = -self.ANGULAR_SPEED * 0.5
        # Otherwise, go straight
        else:
            twist_msg.linear.x = self.LINEAR_SPEED
        
        self.cmd_vel_pub.publish(twist_msg)
        
        # Condition to leave obstacle: is path to original goal clear?
        waypoint = self.global_path[self.current_waypoint_index].pose.position
        angle_to_waypoint = math.atan2(waypoint.y - self.robot_pose.y, waypoint.x - self.robot_pose.x)
        angle_diff = angle_to_waypoint - self.robot_yaw
        if abs(angle_diff) < math.pi / 4 and not self.is_obstacle_in_front():
            rospy.loginfo("Path to goal is clear. Resuming NAVIGATING.")
            self.state = 'NAVIGATING'

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        rospy.loginfo("Hybrid navigator (direct) is running. Set a goal in RViz with '2D Nav Goal'.")

        while not rospy.is_shutdown():
            if self.state == 'NAVIGATING':
                if self.is_obstacle_in_front():
                    self.state = 'OBSTACLE_AVOIDANCE'
                    self.stop_robot() # Stop briefly before changing behavior
                else:
                    self.follow_global_path()
            
            elif self.state == 'OBSTACLE_AVOIDANCE':
                self.avoid_obstacle_bug()

            elif self.state == 'IDLE':
                self.stop_robot()

            rate.sleep()

if __name__ == '__main__':
    try:
        navigator = HybridNavigatorDirect()
        navigator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Hybrid navigator (direct) shut down.") 