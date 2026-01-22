#!/usr/bin/env python
import rospy
import tf
import math
from enum import Enum
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class State(Enum):
    IDLE = 1
    GO_TO_GOAL = 2
    WALL_FOLLOW = 3

class BugNavigator:
    def __init__(self):
        rospy.init_node('bug_navigator')

        # Parameters
        self.linear_speed = rospy.get_param('~linear_speed', 0.3)
        self.angular_speed = rospy.get_param('~angular_speed', 0.5)
        self.obstacle_threshold = rospy.get_param('~obstacle_threshold', 0.5)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.2)

        # Publishers and Subscribers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        # State variables
        self.state = State.IDLE
        self.current_pose = None
        self.goal_pose = None
        self.scan_data = None
        
        rospy.loginfo("Bug Navigator node started. Waiting for goal...")
        rospy.Timer(rospy.Duration(0.1), self.control_loop) # 10 Hz control loop

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        self.scan_data = msg

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.state = State.GO_TO_GOAL
        rospy.loginfo(f"New goal received: ({self.goal_pose.position.x:.2f}, {self.goal_pose.position.y:.2f}). State: GO_TO_GOAL")

    def get_heading_and_distance(self):
        if not self.current_pose or not self.goal_pose:
            return None, None
        
        # Get yaw from quaternion
        q = self.current_pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        heading_error = angle_to_goal - yaw

        # Normalize heading error to [-pi, pi]
        if heading_error > math.pi: heading_error -= 2 * math.pi
        if heading_error < -math.pi: heading_error += 2 * math.pi
        
        return heading_error, distance

    def check_for_obstacle(self):
        if not self.scan_data: return False
        
        # Check a narrow cone in front of the robot
        num_readings = len(self.scan_data.ranges)
        center_index = num_readings / 2
        cone_width = 10 # Check 10 readings on each side of the center
        
        for i in range(center_index - cone_width, center_index + cone_width):
            if self.scan_data.ranges[i] < self.obstacle_threshold:
                rospy.logwarn(f"Obstacle detected! Distance: {self.scan_data.ranges[i]:.2f}m. Switching to WALL_FOLLOW.")
                return True
        return False

    def control_loop(self, event):
        if self.state == State.IDLE:
            self.stop_robot()
            return

        if not self.current_pose or not self.scan_data:
            rospy.loginfo_throttle(5, "Waiting for odometry and scan data...")
            return
            
        heading_error, distance = self.get_heading_and_distance()
        if distance is None: return

        # Check if goal is reached
        if distance < self.goal_tolerance:
            rospy.loginfo("Goal Reached! Switching to IDLE.")
            self.state = State.IDLE
            self.stop_robot()
            return

        # --- STATE MACHINE ---
        if self.state == State.GO_TO_GOAL:
            if self.check_for_obstacle():
                self.state = State.WALL_FOLLOW
            else:
                self.go_to_goal_behavior(heading_error)
        
        elif self.state == State.WALL_FOLLOW:
            # Simple heuristic: if goal is clear, switch back
            if not self.check_for_obstacle() and abs(heading_error) < 0.3: # about 17 degrees
                 rospy.loginfo("Path to goal is clear. Switching to GO_TO_GOAL.")
                 self.state = State.GO_TO_GOAL
            else:
                 self.wall_follow_behavior()
    
    def go_to_goal_behavior(self, heading_error):
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed
        twist_msg.angular.z = 1.0 * heading_error # Proportional controller
        self.cmd_pub.publish(twist_msg)

    def wall_follow_behavior(self):
        # A simple wall-following logic: turn left if obstacle is ahead,
        # otherwise try to turn slightly right to stay close to the wall.
        twist_msg = Twist()
        if self.check_for_obstacle():
            # Obstacle is right in front, turn left sharply
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = self.angular_speed
        else:
            # Path is clear, move forward and turn slightly right to find the wall again
            twist_msg.linear.x = self.linear_speed * 0.5
            twist_msg.angular.z = -self.angular_speed * 0.5
        self.cmd_pub.publish(twist_msg)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

if __name__ == '__main__':
    try:
        navigator = BugNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass