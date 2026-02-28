#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import time

class MuxerTester:
    def __init__(self):
        rospy.init_node('muxer_tester', anonymous=False)
        
        # Publisher for testing
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_nav', Twist, queue_size=1)
        
        # Wait for muxer service
        rospy.wait_for_service('/cmd_vel_mux/select', timeout=5.0)
        self.mux_select = rospy.ServiceProxy('/cmd_vel_mux/select', Empty)
        
        rospy.loginfo("Muxer tester initialized. Testing velocity control...")
        
    def test_muxer(self):
        """Test that we can switch to our navigator and publish commands."""
        try:
            # Switch to our navigator
            self.mux_select()
            rospy.loginfo("Successfully switched to navigator control")
            
            # Publish a simple command
            twist = Twist()
            twist.linear.x = 0.1
            twist.angular.z = 0.0
            
            self.cmd_vel_pub.publish(twist)
            rospy.loginfo("Published test command to /cmd_vel_nav")
            
            # Wait a bit
            time.sleep(2.0)
            
            # Stop the robot
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)
            rospy.loginfo("Published stop command")
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to test muxer: {e}")
    
    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            self.test_muxer()
            rate.sleep()

if __name__ == '__main__':
    try:
        tester = MuxerTester()
        tester.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Muxer tester shut down.") 