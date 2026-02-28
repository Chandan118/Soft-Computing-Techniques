#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from actionlib_msgs.msg import GoalID
import math
import numpy as np  
from pybind11_rdp import rdp


class HybridNavigator:
    def __init__(self):
        rospy.init_node('hybrid_navigator', anonymous=False)

        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Robot state variables
        self.robot_pose = None
        self.robot_yaw = None
        self.laser_scan_data = None
        
        # Navigation state machine
        self.state = 'IDLE'
        self.global_path = None
        self.current_waypoint_index = 0
        
        # --- TUNABLE PARAMETERS ---
        self.GOAL_RADIUS = 0.15  # meters - reduced from 0.2 to prevent immediate waypoint completion
        self.OBSTACLE_THRESHOLD = 0.8 # meters, how close an obstacle can be
        self.LINEAR_SPEED = 0.2  # m/s
        self.ANGULAR_SPEED = 0.5 # rad/s
        self.WALL_FOLLOW_DISTANCE = 0.8 # meters

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.robot_yaw = yaw

    def scan_callback(self, msg):
        self.laser_scan_data = msg
        

    def is_obstacle_in_front(self):
        if self.laser_scan_data is None: return False
        
        # Check a 60-degree arc in front of the robot
        ranges = self.laser_scan_data.ranges
        angle_increment = self.laser_scan_data.angle_increment
        angle_min = self.laser_scan_data.angle_min
        
        # Calculate the index of the front direction (0 degrees)
        front_index = int((0 - angle_min) / angle_increment)
        
        # Calculate the index range for 30 degrees (half of 60 degrees)
        index_range = int((30 * np.pi / 180) / angle_increment)
        
        # Get the start and end indices for the 60-degree arc in front of the robot
        start_index = max(0, front_index - index_range)
        end_index = min(len(ranges), front_index + index_range)
        
        # Find minimum distance without creating intermediate lists
        min_distance = float('inf')
        for i in range(start_index, end_index):
            r = ranges[i]
            if 0.01 < r < min_distance:
                min_distance = r
        
        # If no valid readings found, min_distance will still be inf
        if min_distance == float('inf'):
            return False
            
        if min_distance < self.OBSTACLE_THRESHOLD:
            rospy.loginfo(f"Obstacle detected at {min_distance:.2f}m. Switching to OBSTACLE_AVOIDANCE.")
            return True
        rospy.loginfo(f"Obstacle detected at {min_distance:.2f}m. keep moving.")
        return False


    def follow_global_path(self):
        if len(self.global_path)==0 or self.robot_pose is None:
            return

        # Get current waypoint
        waypoint = self.global_path[self.current_waypoint_index]
        
        # Check if waypoint is reached
        distance_to_waypoint = math.sqrt((waypoint[0] - self.robot_pose.x)**2 + (waypoint[1] - self.robot_pose.y)**2)
        
        # DEBUG: Print position info
        rospy.loginfo_throttle(1, f"Robot: ({self.robot_pose.x:.3f}, {self.robot_pose.y:.3f}), "
                                  f"Waypoint {self.current_waypoint_index}: ({waypoint[0]:.3f}, {waypoint[1]:.3f}), "
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
        angle_to_waypoint = math.atan2(waypoint[1] - self.robot_pose.y, waypoint[0] - self.robot_pose.x)
        angle_diff = angle_to_waypoint - self.robot_yaw
        
        # Normalize angle to be between -pi and pi
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        if angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Proportional control
        twist_msg = Twist()
        if abs(angle_diff) > 0.3:  # 0.3 radians (~17 degrees)
            twist_msg.angular.z = 0.1*self.ANGULAR_SPEED * (1 if angle_diff > 0 else -1)
            twist_msg.linear.x = 0.1
        else:
            twist_msg.linear.x = self.LINEAR_SPEED
            twist_msg.angular.z = 0
            
        # DEBUG: Print what we're commanding
        rospy.loginfo_throttle(2, f"Commanding: linear={twist_msg.linear.x:.3f}, angular={twist_msg.angular.z:.3f}")
        
        self.cmd_vel_pub.publish(twist_msg)
    def avoid_obstacle_bug(self):
        """A simple Bug-like algorithm: turn and follow the wall."""
        rospy.loginfo("Entering avoid_obstacle_bug function")
        if self.laser_scan_data is None: 
            rospy.logdebug("No laser scan data available")
            return

        ranges = self.laser_scan_data.ranges
        twist_msg = Twist()
        # Basic wall following: try to keep obstacle on the left
        # Get ranges for front and left-front
        front_dist = ranges[0]
        left_front_dist = ranges[45]
        rospy.loginfo(f"Distance measurements - Front: {front_dist:.3f}, Left-front: {left_front_dist:.3f}")

        # Condition to leave obstacle: is path to original goal clear?
        waypoint = self.global_path[self.current_waypoint_index]
        angle_to_waypoint = math.atan2(waypoint[1] - self.robot_pose.y, waypoint[0] - self.robot_pose.x)
        angle_diff = angle_to_waypoint - self.robot_yaw
        obstacle_in_front = self.is_obstacle_in_front()
        rospy.loginfo(f"Exit condition check - Angle diff: {abs(angle_diff):.3f}, Obstacle in front: {obstacle_in_front}")
        if abs(angle_diff) < math.pi / 3 and not obstacle_in_front:
            rospy.loginfo("Path to goal is clear. Resuming NAVIGATING.")
            self.state = 'NAVIGATING'
            return


        # If too close in front, turn right
        if front_dist < self.WALL_FOLLOW_DISTANCE:
            #twist_msg.linear.x = 0.05
            twist_msg.angular.z = self.ANGULAR_SPEED
            rospy.loginfo("Too close in front - Turning right slowly")
        # If space on the left, turn towards it
        elif left_front_dist > self.WALL_FOLLOW_DISTANCE * 0.8:
            #twist_msg.linear.x = self.LINEAR_SPEED
            twist_msg.angular.z = -self.ANGULAR_SPEED * 0.5
            rospy.loginfo("Space on the left - Turning towards it")
        # If too close on the left, turn away
        elif left_front_dist < self.WALL_FOLLOW_DISTANCE * 0.8:
            #twist_msg.linear.x = self.LINEAR_SPEED
            twist_msg.angular.z = self.ANGULAR_SPEED * 0.5
            rospy.loginfo("Too close on the left - Turning away")
        # Otherwise, go straight
        else:
            twist_msg.linear.x = self.LINEAR_SPEED
            rospy.logdebug("Clear path - Moving straight")
        
        rospy.logdebug(f"Publishing cmd_vel: linear.x={twist_msg.linear.x:.3f}, angular.z={twist_msg.angular.z:.3f}")
        self.cmd_vel_pub.publish(twist_msg)
        


    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        # We need a subscriber to the goal set in RViz
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        rospy.loginfo("Hybrid navigator is running. Set a goal in RViz with '2D Nav Goal'.")

        while not rospy.is_shutdown():
            if self.state == 'NAVIGATING':
                if self.is_obstacle_in_front():
                    self.state = 'OBSTACLE_AVOIDANCE'
                    rospy.loginfo("state: OBSTACLE_AVOIDANCE stop")
                    self.stop_robot() # Stop briefly before changing behavior
                else:
                    rospy.loginfo("state: NAVIGATING")
                    self.follow_global_path()
            
            elif self.state == 'OBSTACLE_AVOIDANCE':
                rospy.loginfo("state: OBSTACLE_AVOIDANCE")
                self.avoid_obstacle_bug()

            elif self.state == 'IDLE':
                self.stop_robot()

            rate.sleep()

    def sample_waypoints(self, waypoints, min_distance=0.15):
        """
        Sample waypoints to reduce path density using RDP algorithm.
        
        Args:
            waypoints (list): List of ROS PoseStamped objects representing the path
            min_distance (float): Minimum distance between waypoints (RDP epsilon parameter)
            
        Returns:
            numpy.ndarray: Simplified path as numpy array of shape (n, 2) where n is the 
                          number of simplified waypoints and each row contains [x, y] coordinates
        """
        if not waypoints:
            return waypoints
            
        # Efficiently convert waypoints using list comprehension and numpy array
        points = np.array([[waypoint.pose.position.x, waypoint.pose.position.y] 
                          for waypoint in waypoints])
        
        # Use pybind11-rdp library for path simplification
        simplified_points = rdp(points, epsilon=min_distance)
        
        # Return numpy array directly instead of converting back to ROS format
        return simplified_points

    def rviz_goal_callback(self, msg):
        """This function is a bit of a hack to integrate with RViz's 2D Nav Goal button."""
        rospy.loginfo("Received a goal from RViz! Requesting path...")
        
        # SOLUTION: Cancel any active move_base goals first
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

    def get_current_orientation(self):
        # Helper to get orientation message
        odom_msg = rospy.wait_for_message('/odom', Odometry)
        return odom_msg.pose.pose.orientation

if __name__ == '__main__':
    # A little more setup is needed here to make the RViz integration robust
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.srv import GetPlan
    from nav_msgs.msg import Path
    
    try:
        navigator = HybridNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass