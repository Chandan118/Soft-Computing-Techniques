#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PathSamplingTester:
    def __init__(self):
        rospy.init_node('path_sampling_tester', anonymous=False)
        rospy.loginfo("Path sampling tester initialized.")
        
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
    
    def test_sampling_distances(self, original_path):
        """Test different sampling distances and show the results."""
        rospy.loginfo("=== Testing Path Sampling ===")
        rospy.loginfo(f"Original path has {len(original_path)} waypoints")
        
        # Calculate total path length
        total_length = 0
        for i in range(1, len(original_path)):
            p1 = original_path[i-1].pose.position
            p2 = original_path[i].pose.position
            segment_length = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
            total_length += segment_length
        
        rospy.loginfo(f"Total path length: {total_length:.2f}m")
        
        # Test different sampling distances
        distances = [0.1, 0.2, 0.3, 0.5, 0.8, 1.0, 1.5]
        
        for min_dist in distances:
            sampled = self.sample_waypoints(original_path, min_dist)
            rospy.loginfo(f"Min distance {min_dist}m: {len(original_path)} -> {len(sampled)} waypoints (reduction: {((len(original_path) - len(sampled)) / len(original_path) * 100):.1f}%)")
            
            # Calculate average distance between waypoints
            if len(sampled) > 1:
                avg_dist = total_length / (len(sampled) - 1)
                rospy.loginfo(f"  Average distance between waypoints: {avg_dist:.2f}m")
        
        rospy.loginfo("=== Sampling Test Complete ===")
    
    def run_test(self):
        """Run a test with a sample path."""
        # Create a sample path (you can replace this with actual path data)
        sample_path = []
        
        # Create a simple path with many close waypoints
        for i in range(20):
            pose = PoseStamped()
            pose.pose.position.x = i * 0.1  # 10cm apart
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0
            sample_path.append(pose)
        
        self.test_sampling_distances(sample_path)

if __name__ == '__main__':
    try:
        tester = PathSamplingTester()
        tester.run_test()
    except rospy.ROSInterruptException:
        rospy.loginfo("Path sampling tester shut down.") 