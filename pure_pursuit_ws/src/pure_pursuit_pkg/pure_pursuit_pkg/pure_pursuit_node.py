#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import csv
import numpy as np

LOOKAHEAD_DISTANCE = 1.20
KP = 1.00

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        
        # Create ROS subscribers and publishers
        self.pub_drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.pub_env_viz = self.create_publisher(Marker, '/env_viz', 10)
        self.pub_dynamic_viz = self.create_publisher(Marker, '/dynamic_viz', 10)
        self.sub_odom = self.create_subscription(Odometry, '/ego_racecar/odom', self.pose_callback, 10)

        # Reading CSV data
        self.xes = []
        self.yes = []
        self.headings = []
        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.id = 0
        self.marker.type = Marker.POINTS
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 0.0
        self.marker.color.b = 1.0

        # Use the absolute path to the CSV file
        file_path = '/sim_ws/src/pure_pursuit_ws/src/pure_pursuit_pkg/pure_pursuit_pkg/data.csv'
        with open(file_path, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                x, y, heading = map(float, row)
                self.xes.append(x)
                self.yes.append(y)
                self.headings.append(heading)
                point = Point()
                point.x = x
                point.y = y
                point.z = 0.0
                self.marker.points.append(point)

        self.angle = 0.0
        self.x_current = 0.0
        self.y_current = 0.0
        self.heading_current = 0.0
        self.current_indx = 0
        self.flag = False

    def pose_callback(self, odometry_info):
        self.x_current = odometry_info.pose.pose.position.x
        self.y_current = odometry_info.pose.pose.position.y
        siny_cosp = 2.0 * (odometry_info.pose.pose.orientation.w * odometry_info.pose.pose.orientation.z + 
                           odometry_info.pose.pose.orientation.x * odometry_info.pose.pose.orientation.y)
        cosy_cosp = 1.0 - 2.0 * (odometry_info.pose.pose.orientation.y * odometry_info.pose.pose.orientation.y + 
                                 odometry_info.pose.pose.orientation.z * odometry_info.pose.pose.orientation.z)
        self.heading_current = np.arctan2(siny_cosp, cosy_cosp)
        
        # TODO: find the current waypoint to track using methods mentioned in lecture

        if not self.flag:
            shortest_distance = 100.0
            for i in range(len(self.xes)):
                distance = (self.xes[i] - self.x_current) ** 2 + (self.yes[i] - self.y_current) ** 2
                if distance < shortest_distance:
                    shortest_distance = distance
                    self.current_indx = i
            self.flag = True

        while np.sqrt((self.xes[self.current_indx] - self.x_current) ** 2 + 
                        (self.yes[self.current_indx] - self.y_current) ** 2) < LOOKAHEAD_DISTANCE:
            self.current_indx += 1
            if self.current_indx >= len(self.xes):
                self.current_indx = 0

        # TODO: transform goal point to vehicle frame of reference

        real_distance = np.sqrt((self.xes[self.current_indx] - self.x_current) ** 2 + 
                                  (self.yes[self.current_indx] - self.y_current) ** 2)
        lookahead_angle = np.arctan2(self.yes[self.current_indx] - self.y_current, self.xes[self.current_indx] - self.x_current)
        del_y = real_distance * np.sin(lookahead_angle - self.heading_current)

        # TODO: calculate curvature/steering angle

        self.angle = KP * 2.0 * del_y / (real_distance ** 2)
        
        # Visualization marker for current goal point

        point = Point()
        marker_2 = Marker()
        point.x = self.xes[self.current_indx]
        point.y = self.yes[self.current_indx]
        point.z = 0.0
        marker_2.points.append(point)
        marker_2.header.frame_id = "map"
        marker_2.id = 0
        marker_2.type = Marker.POINTS
        marker_2.action = Marker.ADD
        marker_2.pose.position.x = 0.0
        marker_2.pose.position.y = 0.0
        marker_2.pose.position.z = 0.0
        marker_2.pose.orientation.x = 0.0
        marker_2.pose.orientation.y = 0.0
        marker_2.pose.orientation.z = 0.0
        marker_2.pose.orientation.w = 1.0
        marker_2.scale.x = 0.2
        marker_2.scale.y = 0.2
        marker_2.color.a = 1.0
        marker_2.color.r = 1.0
        marker_2.color.g = 0.0
        marker_2.color.b = 0.0

        self.reactive_control()
        self.pub_env_viz.publish(self.marker)
        self.pub_dynamic_viz.publish(marker_2)
        #self.get_logger().info('Published marker with current point ({}, {})'.format(point.x, point.y))


    # TODO: publish drive message, don't forget to limit the steering angle.

    def reactive_control(self):
        ackermann_drive_result = AckermannDriveStamped()
        ackermann_drive_result.drive.steering_angle = self.angle
        if abs(self.angle) > np.radians(20.0):
            ackermann_drive_result.drive.speed = 0.5
        elif abs(self.angle) > np.radians(10.0):
            ackermann_drive_result.drive.speed = 1.0
        else:
            ackermann_drive_result.drive.speed = 1.5
        self.pub_drive.publish(ackermann_drive_result)

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
