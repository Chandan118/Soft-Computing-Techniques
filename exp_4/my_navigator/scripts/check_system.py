#!/usr/bin/env python
"""
check_system.py

Author      : Chandan Sheikder
Email       : chandan@bit.edu.cn
Phone       : +8618222390506
Affiliation : Beijing Institute of Technology (BIT)
Date        : 2026-03-23

Description:
    Module for Check System
"""

import rospy
import time
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty

class SystemChecker:
    def __init__(self):
        rospy.init_node('system_checker', anonymous=False)
        rospy.loginfo("System checker initialized. Checking ROS system...")
        
    def check_services(self):
        """Check if required services are available."""
        rospy.loginfo("=== Checking Services ===")
        
        # Check make_plan service
        try:
            rospy.wait_for_service('/move_base/make_plan', timeout=5.0)
            rospy.loginfo("✅ /move_base/make_plan service is available")
        except rospy.ROSException:
            rospy.logerr("❌ /move_base/make_plan service is NOT available")
            return False
        
        # Check muxer service
        try:
            rospy.wait_for_service('/cmd_vel_mux/select', timeout=5.0)
            rospy.loginfo("✅ /cmd_vel_mux/select service is available")
        except rospy.ROSException:
            rospy.logwarn("⚠️  /cmd_vel_mux/select service is NOT available (muxer may not be running)")
        
        return True
    
    def check_topics(self):
        """Check if required topics are available."""
        rospy.loginfo("=== Checking Topics ===")
        
        # Check odometry
        try:
            rospy.wait_for_message('/odom', Odometry, timeout=5.0)
            rospy.loginfo("✅ /odom topic is publishing")
        except rospy.ROSException:
            rospy.logerr("❌ /odom topic is NOT publishing")
            return False
        
        # Check laser scan
        try:
            rospy.wait_for_message('/scan', LaserScan, timeout=5.0)
            rospy.loginfo("✅ /scan topic is publishing")
        except rospy.ROSException:
            rospy.logerr("❌ /scan topic is NOT publishing")
            return False
        
        # Check if we can publish to cmd_vel
        try:
            from geometry_msgs.msg import Twist
            pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            time.sleep(0.1)  # Give time for publisher to initialize
            rospy.loginfo("✅ Can publish to /cmd_vel")
        except Exception as e:
            rospy.logerr(f"❌ Cannot publish to /cmd_vel: {e}")
            return False
        
        return True
    
    def check_move_base_status(self):
        """Check move_base node status."""
        rospy.loginfo("=== Checking move_base Status ===")
        
        try:
            # Try to get a simple plan
            make_plan_service = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
            
            # Get current robot pose
            odom_msg = rospy.wait_for_message('/odom', Odometry, timeout=2.0)
            
            # Create a simple test plan
            start = PoseStamped()
            start.header.stamp = rospy.Time.now()
            start.header.frame_id = 'map'
            start.pose.position = odom_msg.pose.pose.position
            start.pose.orientation = odom_msg.pose.pose.orientation
            
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = 'map'
            goal.pose.position.x = start.pose.position.x + 0.5
            goal.pose.position.y = start.pose.position.y
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0
            
            plan_response = make_plan_service(start=start, goal=goal, tolerance=0.5)
            
            if plan_response.plan.poses:
                rospy.loginfo(f"✅ move_base can generate plans ({len(plan_response.plan.poses)} waypoints)")
            else:
                rospy.logwarn("⚠️  move_base returned empty plan")
                
        except rospy.ServiceException as e:
            rospy.logerr(f"❌ move_base service call failed: {e}")
            return False
        except Exception as e:
            rospy.logerr(f"❌ move_base test failed: {e}")
            return False
        
        return True
    
    def run_checks(self):
        """Run all system checks."""
        rospy.loginfo("Starting system checks...")
        
        services_ok = self.check_services()
        topics_ok = self.check_topics()
        move_base_ok = self.check_move_base_status()
        
        rospy.loginfo("=== Summary ===")
        if services_ok and topics_ok and move_base_ok:
            rospy.loginfo("✅ All systems are working correctly!")
            return True
        else:
            rospy.logerr("❌ Some systems are not working correctly.")
            return False

if __name__ == '__main__':
    try:
        checker = SystemChecker()
        success = checker.run_checks()
        if success:
            rospy.loginfo("System is ready for navigation!")
        else:
            rospy.logerr("System has issues that need to be resolved.")
    except rospy.ROSInterruptException:
        rospy.loginfo("System checker shut down.") 