#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import time

class VelocityTester:
    def __init__(self):
        rospy.init_node('velocity_tester', anonymous=False)
        
        # Publisher for testing
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_nav', Twist, queue_size=1)
        
        # Wait for muxer service
        try:
            rospy.wait_for_service('/cmd_vel_mux/select', timeout=5.0)
            self.mux_select = rospy.ServiceProxy('/cmd_vel_mux/select', Empty)
            rospy.loginfo("Velocity muxer service found")
        except rospy.ROSException:
            rospy.logwarn("Velocity muxer service not found!")
            self.mux_select = None
        
        rospy.loginfo("Velocity tester initialized. Testing robot movement...")
        
    def test_movement(self):
        """Test basic robot movement."""
        try:
            # Switch to our control
            if self.mux_select:
                self.mux_select()
                rospy.loginfo("Switched to navigator control")
            
            # Test 1: Move forward
            rospy.loginfo("Test 1: Moving forward for 3 seconds...")
            twist = Twist()
            twist.linear.x = 0.1  # Slow forward movement
            twist.angular.z = 0.0
            
            start_time = time.time()
            while time.time() - start_time < 3.0 and not rospy.is_shutdown():
                self.cmd_vel_pub.publish(twist)
                rospy.loginfo("Publishing forward velocity")
                rospy.sleep(0.1)
            
            # Test 2: Turn left
            rospy.loginfo("Test 2: Turning left for 2 seconds...")
            twist.linear.x = 0.0
            twist.angular.z = 0.3  # Turn left
            
            start_time = time.time()
            while time.time() - start_time < 2.0 and not rospy.is_shutdown():
                self.cmd_vel_pub.publish(twist)
                rospy.loginfo("Publishing left turn velocity")
                rospy.sleep(0.1)
            
            # Test 3: Turn right
            rospy.loginfo("Test 3: Turning right for 2 seconds...")
            twist.angular.z = -0.3  # Turn right
            
            start_time = time.time()
            while time.time() - start_time < 2.0 and not rospy.is_shutdown():
                self.cmd_vel_pub.publish(twist)
                rospy.loginfo("Publishing right turn velocity")
                rospy.sleep(0.1)
            
            # Stop
            rospy.loginfo("Test 4: Stopping...")
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)
            rospy.loginfo("Published stop command")
            
            rospy.loginfo("Movement test completed!")
            
        except Exception as e:
            rospy.logerr(f"Movement test failed: {e}")
    
    def run(self):
        rate = rospy.Rate(0.2)  # Run every 5 seconds
        while not rospy.is_shutdown():
            self.test_movement()
            rate.sleep()

if __name__ == '__main__':
    try:
        tester = VelocityTester()
        tester.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Velocity tester shut down.") 