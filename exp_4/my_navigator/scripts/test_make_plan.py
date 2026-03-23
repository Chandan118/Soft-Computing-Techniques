#!/usr/bin/env python
"""
test_make_plan.py

Author      : Chandan Sheikder
Email       : chandan@bit.edu.cn
Phone       : +8618222390506
Affiliation : Beijing Institute of Technology (BIT)
Date        : 2026-03-23

Description:
    Module for Test Make Plan
"""

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Odometry
import time

class MakePlanTester:
    def __init__(self):
        rospy.init_node('make_plan_tester', anonymous=False)
        
        # Wait for services and topics
        rospy.loginfo("Waiting for make_plan service...")
        rospy.wait_for_service('/move_base/make_plan', timeout=10.0)
        
        self.make_plan_service = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        
        # Get initial robot pose
        rospy.loginfo("Waiting for robot pose from odometry...")
        try:
            odom_msg = rospy.wait_for_message('/odom', Odometry, timeout=5.0)
            self.robot_pose = odom_msg.pose.pose.position
            self.robot_orientation = odom_msg.pose.pose.orientation
            rospy.loginfo(f"Robot pose: ({self.robot_pose.x:.2f}, {self.robot_pose.y:.2f})")
        except rospy.ROSException:
            rospy.logerr("Could not get robot pose from odometry")
            return
        
        rospy.loginfo("MakePlan tester initialized. Testing service...")
        
    def test_make_plan(self):
        """Test the make_plan service with a simple goal."""
        try:
            # Create start pose
            start = PoseStamped()
            start.header.stamp = rospy.Time.now()
            start.header.frame_id = 'map'
            start.pose.position = self.robot_pose
            start.pose.orientation = self.robot_orientation
            
            # Create a simple goal (1 meter forward)
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = 'map'
            goal.pose.position.x = self.robot_pose.x + 1.0
            goal.pose.position.y = self.robot_pose.y
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            
            rospy.loginfo(f"Testing make_plan from ({start.pose.position.x:.2f}, {start.pose.position.y:.2f}) to ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})")
            
            # Call the service
            plan_response = self.make_plan_service(start=start, goal=goal, tolerance=0.5)
            
            if plan_response.plan.poses:
                rospy.loginfo(f"SUCCESS: Plan received with {len(plan_response.plan.poses)} waypoints")
                for i, pose in enumerate(plan_response.plan.poses[:5]):  # Show first 5 waypoints
                    rospy.loginfo(f"  Waypoint {i}: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
                if len(plan_response.plan.poses) > 5:
                    rospy.loginfo(f"  ... and {len(plan_response.plan.poses) - 5} more waypoints")
            else:
                rospy.logwarn("Plan received but no waypoints in the path")
                
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        except Exception as e:
            rospy.logerr(f"Unexpected error: {e}")
    
    def run(self):
        rate = rospy.Rate(0.5)  # Test every 2 seconds
        while not rospy.is_shutdown():
            self.test_make_plan()
            rate.sleep()

if __name__ == '__main__':
    try:
        tester = MakePlanTester()
        tester.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("MakePlan tester shut down.") 