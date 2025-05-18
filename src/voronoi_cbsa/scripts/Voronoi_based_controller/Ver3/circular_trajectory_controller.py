#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist

class CircularTrajectoryController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('circular_trajectory_controller', anonymous=True)
        
        # Parameters for target_1's circular trajectory
        self.target1_radius = rospy.get_param('~target1/radius', 1.0)        # Radius in meters
        self.target1_angular_velocity = rospy.get_param('~target1/angular_velocity', 0.5)  # rad/s
        
        # Parameters for target_2's circular trajectory
        self.target2_radius = rospy.get_param('~target2/radius', 1.5)        # Radius in meters
        self.target2_angular_velocity = rospy.get_param('~target2/angular_velocity', 0.4)  # rad/s
        
        # Create publishers for the two targets
        self.target1_pub = rospy.Publisher('/target_1/cmd_vel', Twist, queue_size=10)
        self.target2_pub = rospy.Publisher('/target_2/cmd_vel', Twist, queue_size=10)
        
        # Timer to regularly publish velocity commands
        self.publish_rate = rospy.get_param('~publish_rate', 10.0)  # Hz
        self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.timer_callback)
        
        # Store the starting time
        self.start_time = rospy.Time.now().to_sec()

    def timer_callback(self, event):
        # Calculate current time since start
        current_time = rospy.Time.now().to_sec()
        
        # Create Twist messages for both targets
        # 修正: 移除多餘的self參數
        target1_twist = self.calculate_circular_velocity(self.target1_radius, self.target1_angular_velocity)
        target2_twist = self.calculate_circular_velocity(self.target2_radius, self.target2_angular_velocity)
        
        # Publish the velocity commands
        self.target1_pub.publish(target1_twist)
        self.target2_pub.publish(target2_twist)
        
    def calculate_circular_velocity(self, radius, angular_velocity):
        """
        Calculate velocity commands for circular motion.
        
        For a differential drive robot like TurtleBot, we need to set the linear velocity
        and angular velocity correctly to achieve circular motion.
        The linear velocity is the product of angular velocity and radius.
        """
        twist = Twist()
        
        # 修正: 這裡計算線速度應該是 v = r * ω
        # 而不是 r = ω / radius
        twist.linear.x = radius * angular_velocity  # 線速度 = 半徑 * 角速度
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_velocity
        
        return twist

if __name__ == '__main__':
    try:
        controller = CircularTrajectoryController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass