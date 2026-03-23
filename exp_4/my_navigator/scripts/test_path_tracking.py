#!/usr/bin/env python
"""
test_path_tracking.py

Author      : Chandan Sheikder
Email       : chandan@bit.edu.cn
Phone       : +8618222390506
Affiliation : Beijing Institute of Technology (BIT)
Date        : 2026-03-23

Description:
    Module for Test Path Tracking
"""

import rospy
import math
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion
from actionlib_msgs.msg import GoalID

class PathTrackingTester:
    def __init__(self):
        rospy.init_node('path_tracking_tester', anonymous=False)
        
        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_nav', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        
        # Robot state
        self.robot_pose = None
        self.robot_yaw = None
        self.global_path = []
        self.current_waypoint_index = 0
        self.state = 'IDLE'
        
        # Feedback control parameters
        self.KP_LINEAR = 0.5
        self.KP_ANGULAR = 1.0
        self.KD_ANGULAR = 0.2
        self.MAX_LINEAR_VEL = 0.3
        self.MAX_ANGULAR_VEL = 0.8
        self.LOOKAHEAD_DISTANCE = 0.5
        self.GOAL_RADIUS = 0.3
        
        # Path tracking variables
        self.path_error = 0.0
        self.heading_error = 0.0
        self.last_heading_error = 0.0
        
        # Smooth motion variables
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.velocity_ramp_rate = 0.1
        
        # Wait for muxer service
        try:
            rospy.wait_for_service('/cmd_vel_mux/select', timeout=5.0)
            self.mux_select = rospy.ServiceProxy('/cmd_vel_mux/select', Empty)
            rospy.loginfo("Velocity muxer service found")
        except rospy.ROSException:
            rospy.logwarn("Velocity muxer service not found!")
            self.mux_select = None
        
        rospy.loginfo("Path tracking tester initialized. Set a goal in RViz to test feedback control.")

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        (_, _, self.robot_yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def rviz_goal_callback(self, msg):
        rospy.loginfo("Goal received! Testing improved path tracking...")
        if self.robot_pose is None:
            rospy.logwarn("Robot pose not available")
            return
        
        # Switch to our control
        if self.mux_select:
            try:
                self.mux_select()
                rospy.loginfo("Switched to path tracking control")
            except rospy.ServiceException as e:
                rospy.logwarn(f"Failed to switch muxer: {e}")
        
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
        
        # Get path from move_base
        try:
            make_plan_service = rospy.ServiceProxy('/move_base/NavfnROS/make_plan', GetPlan)
            
            start = PoseStamped()
            start.header.stamp = rospy.Time(0)  # Use latest available transform
            start.header.frame_id = 'map'
            start.pose.position = self.robot_pose
            start.pose.orientation = self.get_current_orientation()
            
            goal = msg
            goal.header.stamp = rospy.Time(0)  # Use latest available transform
            
            plan_response = make_plan_service(start=start, goal=goal, tolerance=0.5)
            
            if plan_response.plan.poses:
                # Sample waypoints to reduce density
                original_count = len(plan_response.plan.poses)
                self.global_path = self.sample_waypoints(plan_response.plan.poses, min_distance=0.15)
                self.current_waypoint_index = 0
                self.state = 'NAVIGATING'
                rospy.loginfo(f"Path received: {original_count} -> {len(self.global_path)} waypoints")
            else:
                rospy.logwarn("No path generated")
                self.state = 'IDLE'
                
        except Exception as e:
            rospy.logerr(f"Failed to get path: {e}")
            self.state = 'IDLE'

    def get_current_orientation(self):
        # Helper to get orientation message
        odom_msg = rospy.wait_for_message('/odom', Odometry)
        return odom_msg.pose.pose.orientation

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

    def find_path_reference_points(self):
        """Find closest point and lookahead point on path."""
        if not self.global_path:
            return None, None
        
        robot_pos = self.robot_pose
        closest_point = None
        closest_distance = float('inf')
        closest_index = 0
        
        for i, pose in enumerate(self.global_path):
            path_point = pose.pose.position
            distance = math.sqrt((path_point.x - robot_pos.x)**2 + (path_point.y - robot_pos.y)**2)
            
            if distance < closest_distance:
                closest_distance = distance
                closest_point = path_point
                closest_index = i
        
        lookahead_point = None
        for i in range(closest_index, len(self.global_path)):
            path_point = self.global_path[i].pose.position
            distance = math.sqrt((path_point.x - robot_pos.x)**2 + (path_point.y - robot_pos.y)**2)
            
            if distance >= self.LOOKAHEAD_DISTANCE:
                lookahead_point = path_point
                break
        
        if lookahead_point is None and self.global_path:
            lookahead_point = self.global_path[-1].pose.position
        
        return closest_point, lookahead_point

    def calculate_path_errors(self, lookahead_point):
        """Calculate path tracking errors."""
        if lookahead_point is None:
            return
        
        desired_heading = math.atan2(lookahead_point.y - self.robot_pose.y, 
                                   lookahead_point.x - self.robot_pose.x)
        
        self.heading_error = self.normalize_angle(desired_heading - self.robot_yaw)
        
        distance_to_lookahead = math.sqrt((lookahead_point.x - self.robot_pose.x)**2 + 
                                        (lookahead_point.y - self.robot_pose.y)**2)
        
        self.path_error = distance_to_lookahead * math.sin(abs(self.heading_error))

    def calculate_control_outputs(self):
        """Calculate control outputs using feedback control."""
        linear_vel = self.MAX_LINEAR_VEL * (1.0 - abs(self.heading_error) / math.pi)
        linear_vel = max(0.1, linear_vel)
        
        angular_vel = (self.KP_ANGULAR * self.heading_error + 
                      self.KD_ANGULAR * (self.heading_error - self.last_heading_error))
        angular_vel += self.KP_LINEAR * self.path_error
        angular_vel = max(-self.MAX_ANGULAR_VEL, min(self.MAX_ANGULAR_VEL, angular_vel))
        
        self.last_heading_error = self.heading_error
        return linear_vel, angular_vel

    def smooth_velocity_commands(self, target_linear, target_angular):
        """Apply smooth velocity ramping."""
        if target_linear > self.current_linear_vel:
            self.current_linear_vel = min(target_linear, self.current_linear_vel + self.velocity_ramp_rate)
        else:
            self.current_linear_vel = max(target_linear, self.current_linear_vel - self.velocity_ramp_rate)
        
        angular_ramp_rate = self.velocity_ramp_rate * 2.0
        if target_angular > self.current_angular_vel:
            self.current_angular_vel = min(target_angular, self.current_angular_vel + angular_ramp_rate)
        else:
            self.current_angular_vel = max(target_angular, self.current_angular_vel - angular_ramp_rate)
        
        return self.current_linear_vel, self.current_angular_vel

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def follow_path(self):
        """Follow the path using feedback control."""
        if not self.global_path or self.robot_pose is None or self.robot_yaw is None:
            return
        
        # Find reference points
        closest_point, lookahead_point = self.find_path_reference_points()
        if closest_point is None or lookahead_point is None:
            return
        
        # Calculate errors
        self.calculate_path_errors(lookahead_point)
        
        # Check waypoint reached
        waypoint = self.global_path[self.current_waypoint_index].pose.position
        distance_to_waypoint = math.sqrt((waypoint.x - self.robot_pose.x)**2 + (waypoint.y - self.robot_pose.y)**2)
        
        if distance_to_waypoint < self.GOAL_RADIUS:
            rospy.loginfo(f"Waypoint {self.current_waypoint_index} reached!")
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.global_path):
                rospy.loginfo("Goal reached!")
                self.state = 'IDLE'
                self.stop_robot()
                return
        
        # Calculate and apply control
        linear_vel, angular_vel = self.calculate_control_outputs()
        linear_vel, angular_vel = self.smooth_velocity_commands(linear_vel, angular_vel)
        
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist_msg)
        
        rospy.loginfo(f"Feedback control: linear={linear_vel:.2f}, angular={angular_vel:.2f}, path_error={self.path_error:.2f}, heading_error={self.heading_error:.2f}")

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.state == 'NAVIGATING':
                self.follow_path()
            elif self.state == 'IDLE':
                self.stop_robot()
            rate.sleep()

if __name__ == '__main__':
    try:
        tester = PathTrackingTester()
        tester.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Path tracking tester shut down.") 