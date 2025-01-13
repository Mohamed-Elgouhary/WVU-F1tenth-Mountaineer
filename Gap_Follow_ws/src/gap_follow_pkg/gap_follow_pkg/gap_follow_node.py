#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class GapFollowNode(Node):
    """ 
    Implement Gap Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('gap_follow_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # Subscribe to LIDAR
        self.subscription = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        # Publish to drive
        self.publisher_ = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        self.angle = 0.0
        self.ranges = []

    def preprocess_lidar(self, ranges, angle_min, angle_increment):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1. Setting each value to the mean over some window
            2. Rejecting high values (eg. > 3m)
        """
        proc_ranges = np.array(ranges)
        min_angle = np.radians(-70)
        max_angle = np.radians(70)
        min_indx = int(np.floor((min_angle - angle_min) / angle_increment))
        max_indx = int(np.ceil((max_angle - angle_min) / angle_increment))

        for i in range(min_indx, max_indx + 1):
            if np.isinf(ranges[i]) or np.isnan(ranges[i]):
                proc_ranges[i] = 0.0
            elif ranges[i] > 3.0:  # Assuming 3m as the threshold for high values
                proc_ranges[i] = 3.0

        self.min_indx = min_indx
        self.max_indx = max_indx
        return proc_ranges, min_indx, max_indx

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges """
        min_indx = self.min_indx
        max_indx = self.max_indx

        start = min_indx
        end = min_indx
        current_start = min_indx - 1
        longest_duration = 0

        for i in range(min_indx, max_indx + 1):
            if current_start < min_indx:
                if free_space_ranges[i] > 0.0:
                    current_start = i
            elif free_space_ranges[i] <= 0.0:
                duration = i - current_start
                if duration > longest_duration:
                    longest_duration = duration
                    start = current_start
                    end = i - 1
                current_start = min_indx - 1

        if current_start >= min_indx:
            duration = max_indx + 1 - current_start
            if duration > longest_duration:
                longest_duration = duration
                start = current_start
                end = max_indx
        self.proc_ranges = free_space_ranges
        return start, end
    
    def find_best_point(self, start_i, end_i, ranges, angle_min, angle_increment):
        """Start_i & end_i are start and end indices of max-gap range, respectively
        Return index of best point in ranges
        Naive: Choose the furthest point within ranges and go there
        """
        current_max = 0.0
        best_point = start_i
        proc_ranges = self.proc_ranges
        for i in range(start_i, end_i + 1):
            if proc_ranges[i] > current_max:
                current_max = proc_ranges[i]
                self.angle = angle_min + i * angle_increment
            elif proc_ranges[i] == current_max:
                if abs(angle_min + i * angle_increment) < abs(self.angle):
                    self.angle = angle_min + i * angle_increment

        return self.angle

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message """
        ranges = data.ranges
        angle_min = data.angle_min
        angle_increment = data.angle_increment
        proc_ranges, min_indx, max_indx = self.preprocess_lidar(ranges, angle_min, angle_increment)
        
        # Find closest point to LiDAR
        closest_indx = min_indx
        closest_distance = data.range_max * 5
        for i in range(min_indx, max_indx + 1):
            distance = np.sum(proc_ranges[i - 2:i + 3])
            if distance < closest_distance:
                closest_distance = distance
                closest_indx = i

        # Dynamic safety bubble size based on speed
        base_speed = 2.5  # Base speed
        min_distance_to_wall = np.min(proc_ranges[min_indx:max_indx + 1])
        if min_distance_to_wall < 0.5:
            speed = 0.5
            radius = 300  # Increase safety margin when very close to walls
        elif min_distance_to_wall < 1.0:
            speed = 1.0
            radius = 200  # Moderate safety margin
        else:
            speed = base_speed
            radius = 100  # Normal safety margin

        # Eliminate all points inside 'bubble' (set them to zero)
        for i in range(closest_indx - radius, closest_indx + radius + 1):
            if i >= 0 and i < len(proc_ranges):  # Ensure indices are within bounds
                proc_ranges[i] = 0.0

        # Find max length gap
        start, end = self.find_max_gap(proc_ranges)

        # Find the best point in the gap
        best_angle = self.find_best_point(start, end, proc_ranges, angle_min, angle_increment)

        # Increase speed if the best angle is close to zero (straight ahead)
        if abs(best_angle) < np.radians(10):  # Threshold for straight route
            speed = 4.0  # Increased speed for straight routes

        # Smooth the steering angles to avoid abrupt changes
        self.angle = 0.8 * self.angle + 0.2 * best_angle

        # Publish Drive message
        ackermann_drive_result = AckermannDriveStamped()
        ackermann_drive_result.drive.steering_angle = self.angle
        ackermann_drive_result.drive.speed = speed

        self.publisher_.publish(ackermann_drive_result)
        self.get_logger().info(f"Steering Angle: {self.angle}, Speed: {ackermann_drive_result.drive.speed}")

def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    gap_follow_node = GapFollowNode()
    rclpy.spin(gap_follow_node)

    gap_follow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
