#!/usr/bin/env python
import rospy
import tf
import math
import random
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Quaternion, Twist

class RobotSimulator:
    def __init__(self):
        rospy.init_node('robot_simulator')

        # Publishers
        self.odom_pub = rospy.Publisher('/noisy_odom', Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Subscriber to get velocity commands from the navigator
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        
        # Noise parameters
        self.odom_pos_noise = 0.01
        self.odom_yaw_noise = 0.01
        self.imu_yaw_noise = 0.02
        self.imu_vel_noise = 0.02

        # Virtual Obstacle (a vertical wall)
        self.wall_x = 2.0
        self.wall_y_start = -2.0
        self.wall_y_end = 2.0

        self.rate = rospy.Rate(50)
        self.last_time = rospy.Time.now()

        rospy.loginfo("Enhanced robot simulator started. Publishing odometry, IMU, and LaserScan.")
        rospy.loginfo(f"Virtual wall placed at x = {self.wall_x} from y = {self.wall_y_start} to {self.wall_y_end}")

    def cmd_vel_callback(self, msg):
        self.linear_speed = msg.linear.x
        self.angular_speed = msg.angular.z

    def run(self):
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            if dt == 0: continue

            # Update perfect ground truth state from cmd_vel
            self.x += self.linear_speed * math.cos(self.theta) * dt
            self.y += self.linear_speed * math.sin(self.theta) * dt
            self.theta += self.angular_speed * dt

            # Publish ground truth TF for visualization
            self.tf_broadcaster.sendTransform(
                (self.x, self.y, 0),
                tf.transformations.quaternion_from_euler(0, 0, self.theta),
                current_time,
                "base_link_ground_truth",
                "odom"
            )

            # --- Publish Noisy Odometry ---
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"
            noisy_x = self.x + random.gauss(0, self.odom_pos_noise)
            noisy_y = self.y + random.gauss(0, self.odom_pos_noise)
            noisy_theta = self.theta + random.gauss(0, self.odom_yaw_noise)
            noisy_q = tf.transformations.quaternion_from_euler(0, 0, noisy_theta)
            odom_msg.pose.pose.position.x = noisy_x
            odom_msg.pose.pose.position.y = noisy_y
            odom_msg.pose.pose.orientation = Quaternion(*noisy_q)
            self.odom_pub.publish(odom_msg)

            # --- Publish Noisy IMU ---
            imu_msg = Imu()
            imu_msg.header.stamp = current_time
            imu_msg.header.frame_id = "base_link"
            imu_noisy_theta = self.theta + random.gauss(0, self.imu_yaw_noise)
            imu_q = tf.transformations.quaternion_from_euler(0, 0, imu_noisy_theta)
            imu_msg.orientation = Quaternion(*imu_q)
            imu_msg.angular_velocity.z = self.angular_speed + random.gauss(0, self.imu_vel_noise)
            self.imu_pub.publish(imu_msg)

            # --- Publish Simulated LaserScan ---
            self.publish_fake_scan(current_time)

            self.last_time = current_time
            self.rate.sleep()

    def publish_fake_scan(self, current_time):
        scan_msg = LaserScan()
        scan_msg.header.stamp = current_time
        scan_msg.header.frame_id = "base_link"
        scan_msg.angle_min = -math.pi
        scan_msg.angle_max = math.pi
        scan_msg.angle_increment = math.pi / 180.0  # 360 readings
        scan_msg.time_increment = 0.0
        scan_msg.range_min = 0.1
        scan_msg.range_max = 10.0
        scan_msg.ranges = [scan_msg.range_max] * 360

        # Calculate if any scan beam hits the wall
        for i in range(360):
            scan_angle = scan_msg.angle_min + i * scan_msg.angle_increment
            global_angle = self.theta + scan_angle

            # Simple ray-casting for a vertical wall
            if math.cos(global_angle) > 0: # Ray is pointing towards the wall
                dist_to_wall = (self.wall_x - self.x) / math.cos(global_angle)
                if 0 < dist_to_wall < scan_msg.range_max:
                    y_hit = self.y + dist_to_wall * math.sin(global_angle)
                    if self.wall_y_start <= y_hit <= self.wall_y_end:
                        scan_msg.ranges[i] = dist_to_wall

        self.scan_pub.publish(scan_msg)

if __name__ == '__main__':
    try:
        simulator = RobotSimulator()
        simulator.run()
    except rospy.ROSInterruptException:
        pass