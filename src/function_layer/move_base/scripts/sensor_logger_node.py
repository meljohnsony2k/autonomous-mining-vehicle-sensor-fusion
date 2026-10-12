#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import csv
import os
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from datetime import datetime

class SensorLogger:
    def __init__(self):
        rospy.init_node('sensor_logger_node')

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.folder = os.path.expanduser("~/sensor_logs")
        os.makedirs(self.folder, exist_ok=True)

        self.imu_file = open(os.path.join(self.folder, f"imu_{timestamp}.csv"), 'w')
        self.filtered_gps_file = open(os.path.join(self.folder, f"filtered_gps_{timestamp}.csv"), 'w')
        self.raw_gps_file = open(os.path.join(self.folder, f"raw_gps_{timestamp}.csv"), 'w')
        self.lidar_file = open(os.path.join(self.folder, f"amcl_{timestamp}.csv"), 'w')
        self.odom_gps_file = open(os.path.join(self.folder, f"odometry_gps_{timestamp}.csv"), 'w')
        self.odom_filtered_file = open(os.path.join(self.folder, f"odometry_filtered_{timestamp}.csv"), 'w')

        self.imu_writer = csv.writer(self.imu_file)
        self.filtered_gps_writer = csv.writer(self.filtered_gps_file)
        self.raw_gps_writer = csv.writer(self.raw_gps_file)
        self.lidar_writer = csv.writer(self.lidar_file)
        self.odom_gps_writer = csv.writer(self.odom_gps_file)
        self.odom_filtered_writer = csv.writer(self.odom_filtered_file)

        # Titles
        self.imu_writer.writerow(['time', 'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                                  'angular_vel_x', 'angular_vel_y', 'angular_vel_z',
                                  'linear_acc_x', 'linear_acc_y', 'linear_acc_z'])
        self.filtered_gps_writer.writerow(['time', 'latitude', 'longitude', 'altitude', 'cov_x', 'cov_y', 'cov_z'])
        self.raw_gps_writer.writerow(['time', 'latitude', 'longitude', 'altitude'])
        self.lidar_writer.writerow(['time', 'x', 'y', 'orientation_z'])
        self.odom_gps_writer.writerow(['time', 'pos_x', 'pos_y'])
        self.odom_filtered_writer.writerow(['time', 'pos_x', 'pos_y', 'orientation_z'])

        # Subscribers
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        rospy.Subscriber('/filtered_navsat/fix', NavSatFix, self.filtered_gps_callback)
        rospy.Subscriber('/navsat/fix', NavSatFix, self.raw_gps_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
        rospy.Subscriber('/odometry/gps', Odometry, self.odom_gps_callback)
        rospy.Subscriber('/odometry/filtered', Odometry, self.odom_filtered_callback)

        rospy.loginfo("Sensor Logger Node started. Logging to ~/sensor_logs/")
        rospy.spin()

    def imu_callback(self, msg):
        now = rospy.Time.now().to_sec()
        self.imu_writer.writerow([
            now,
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        ])
        self.imu_file.flush()

    def filtered_gps_callback(self, msg):
        now = rospy.Time.now().to_sec()
        self.filtered_gps_writer.writerow([
            now,
            msg.latitude, msg.longitude, msg.altitude,
            msg.position_covariance[0], msg.position_covariance[4], msg.position_covariance[8]
        ])
        self.filtered_gps_file.flush()

    def raw_gps_callback(self, msg):
        now = rospy.Time.now().to_sec()
        self.raw_gps_writer.writerow([
            now,
            msg.latitude, msg.longitude, msg.altitude
        ])
        self.raw_gps_file.flush()

    def amcl_pose_callback(self, msg):
        now = rospy.Time.now().to_sec()
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        self.lidar_writer.writerow([
            now, pos.x, pos.y, orient.z
        ])
        self.lidar_file.flush()


    def odom_gps_callback(self, msg):
        now = rospy.Time.now().to_sec()
        pos = msg.pose.pose.position
        self.odom_gps_writer.writerow([
            now, pos.x, pos.y
        ])
        self.odom_gps_file.flush()

    def odom_filtered_callback(self, msg):
        now = rospy.Time.now().to_sec()
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        self.odom_filtered_writer.writerow([
            now, pos.x, pos.y, orient.z
        ])
        self.odom_filtered_file.flush()

    def __del__(self):
        self.imu_file.close()
        self.filtered_gps_file.close()
        self.raw_gps_file.close()
        self.lidar_file.close()
        self.odom_gps_file.close()
        self.odom_filtered_file.close()

if __name__ == '__main__':
    try:
        SensorLogger()
    except rospy.ROSInterruptException:
        pass
