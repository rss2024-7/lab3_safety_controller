#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker

from safety_controller.visualization_tools import VisualizationTools


class SafetyController(Node):

    def __init__(self):
        super().__init__("safety_controller")
        # Declare parameters to make them available for use
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/drive")
        self.VELOCITY = 4.0

        # Fetch constants from the ROS parameter server
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        
        self.WALL_TOPIC = "/wall"

        self.drive_msg = None


	    # Initialize your publishers and subscribers here
        self.publisher_ = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        self.subscription = self.create_subscription(
            LaserScan,
            self.SCAN_TOPIC,
            self.lidar_callback,
            10)
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            self.DRIVE_TOPIC,
            self.drive_callback,
            10)
        
        # self.line_pub = self.create_publisher(Marker, self.WALL_TOPIC, 1)
        # self.dist_line_pub = self.create_publisher(Marker, self.DISTANCE_TOPIC, 1)
        # self.angle_pub = self.create_publisher(Marker, self.ANGLE_TOPIC, 1)

        self.CAR_LENGTH = 0.3 # meters
        self.CAR_WIDTH = 0.32 # meters (not sure if value is correct, just guessing)

        self.drive_forward()

    # Write your callback functions here 
    def drive_forward(self):
        self.get_logger().info('Drive Forward')
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.VELOCITY
        drive_msg.drive.acceleration = 0.0
        drive_msg.drive.jerk = 0.0

        drive_msg.drive.steering_angle = 0.0
        drive_msg.drive.steering_angle_velocity = 0.0
        self.publisher_.publish(drive_msg)

    def stop(self):
        # self.get_logger().info('STOP')
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        if self.drive_msg:
            msg.drive.steering_angle = self.drive_msg.drive.steering_angle
        self.publisher_.publish(msg)



    def lidar_callback(self, scan):
        # self.get_logger().info('Received scan!')
        MAX_STEER = 0.34
        FUTURE_DIST = 0.03
        steering_angle = 0.0
        if self.drive_msg:
            FUTURE_DIST += 0.01 * self.drive_msg.drive.speed
            steering_angle = self.drive_msg.drive.steering_angle


        ranges = np.array(scan.ranges)
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment

        angles = np.array([angle_min + angle_increment * i for i in range(len(ranges))])

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        low_y = - self.CAR_WIDTH / 2 + 0.02 * min(0, steering_angle / MAX_STEER)
        high_y = self.CAR_WIDTH / 2 + 0.02 * max(0, steering_angle / MAX_STEER)
        low_x = 0
        high_x = 4.0 * FUTURE_DIST

        within_front = np.where(np.logical_and(y >= low_y, y <= high_y))
        x_within = x[within_front]
        
        close_front = np.logical_and(x_within >= low_x, x_within <= high_x)

        if np.any(close_front):
            while True:
                self.stop()


    def drive_callback(self, drive):
        self.drive_msg = drive

def main():

    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
