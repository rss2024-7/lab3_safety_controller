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
        self.declare_parameter("drive_topic", "/vesc/high_level/output")
        self.declare_parameter("stop_topic", "/vesc/low_level/input/safety")
        self.speed = 1.0

        # Fetch constants from the ROS parameter server
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.STOP_TOPIC = self.get_parameter('stop_topic').get_parameter_value().string_value
        
        self.WALL_TOPIC = "/wall"

        self.drive_msg = None
            
        self.DRIVE_TOPIC = "/vesc/high_level/ackermann_cmd"
        self.get_logger().info(self.DRIVE_TOPIC)

        # Initialize your publishers and subscribers here
        self.publisher_ = self.create_publisher(AckermannDriveStamped, self.STOP_TOPIC, 10)
        self.subscription = self.create_subscription(
           LaserScan,
           self.SCAN_TOPIC,
           self.lidar_callback,
           10)
        self.subscription2 = self.create_subscription(
           AckermannDriveStamped,
           self.DRIVE_TOPIC,
           self.drive_callback,
           10)
        

        self.CAR_LENGTH = 0.3 # meters
        self.CAR_WIDTH = 0.32 # meters (includes buffer)
        self.DIST_TO_BUMPER = 0.12
        # UNCOMMENT LINE BELOW FOR TESTING
        # self.drive_forward()
        self.get_logger().info("SAFETY CONTROLLER STARTED!")

    # Write your callback functions here 

    # intercept latest drive command
    def drive_callback(self, drive):
        self.drive_msg = drive


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
        # Fetch constants from the ROS parameter server
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value

        # self.get_logger().info('Received scan!')
        MAX_STEER = 0.34
        steering_angle = 0.0

        if self.drive_msg is not None:
            #if self.drive_msg.drive.speed > 0.0:
            #    self.get_logger().info('"%s"' % self.drive_msg.drive.speed)
            # if self.drive_msg.drive.speed > self.SPEED:
            self.speed = self.drive_msg.drive.speed
            steering_angle = self.drive_msg.drive.steering_angle

        #react_time = 0.05
        #max_deceleration = 14.0 # m/s^2
        #stop_dist = 2.0 * self.speed / max_deceleration + react_time * self.speed
        
        # found via experimenting
        stop_dist = (0.41 * self.speed + 0.2) ** 2

        # polar lidar coordinates to cartesian coordinates
        ranges = np.array(scan.ranges)
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment

        angles = np.array([angle_min + angle_increment * i for i in range(len(ranges))])

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # create rectangle in front of car
        low_y = - self.CAR_WIDTH / 2 + (x) * np.arctan(steering_angle)
        high_y = self.CAR_WIDTH / 2 + (x) * np.arctan(steering_angle)
        low_x = self.DIST_TO_BUMPER
        high_x = self.DIST_TO_BUMPER + stop_dist

        # check if any points in rectangle
        within_front = np.where(np.logical_and(np.greater_equal(y, low_y), np.less_equal(y, high_y)))
        x_within = x[within_front]
        close_front = np.logical_and(x_within >= low_x, x_within <= high_x)

        
        # can't create circle if steering angle is exactly 0
        if np.any(close_front):
            self.stop()
            return

        # IF HERE, THERE IS A STEERING ANGLE

        # create circular section in front of car
        turn_radius = np.abs(self.CAR_LENGTH / np.sin(steering_angle))
        turn_sign = np.sign(steering_angle) # -1 for right, 1 for left turn
        inner_radius = turn_radius - self.CAR_WIDTH / 2.0
        outer_radius = turn_radius - self.CAR_WIDTH / 2.0

        inner_circle = turn_sign * inner_radius - turn_sign * np.sqrt(inner_radius**2 - x**2)
        outer_circle = turn_sign * outer_radius - turn_sign * np.sqrt(outer_radius**2 - x**2)

        # end of the circular section
        dist_line = - turn_sign * np.tan(stop_dist / turn_radius) * x + turn_sign * turn_radius
        
        if turn_sign == 1: # LEFT
            upper_side = inner_circle
            lower_side = outer_circle
            within_turn = np.where(np.logical_and(lower_side <= y, y <= upper_side))
            y_within = y[within_turn]
            dist_line = dist_line[within_turn]
            close_front = y_within <= dist_line
        if turn_sign == -1: # RIGHT
            upper_side = outer_circle
            lower_side = inner_circle
            within_turn = np.where(np.logical_and(lower_side <= y, y<= upper_side))
            y_within = y[within_turn]
            dist_line = dist_line[within_turn]
            close_front = y_within >= dist_line

        if np.any(close_front):
            self.stop()
        
        

        if np.any(close_front):
            # UNCOMMENT WHILE LOOP BELOW FOR TESTING WITH WALL FOLLOWER IN SIM
            # while True: # blocking code >:)
            #     self.stop()
            self.stop()

def main():

    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
