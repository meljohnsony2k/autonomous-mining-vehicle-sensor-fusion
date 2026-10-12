#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import random
from sensor_msgs.msg import NavSatFix
from gazebo_msgs.msg import ModelStates

class GPSFilterNode:
    def __init__(self):
        rospy.init_node("gps_filter_node")

        # Map/world frame tunnel limits
        self.tunnel_x_min = -8.0
        self.tunnel_x_max = 8.0
        self.tunnel_y_min = 0.0
        self.tunnel_y_max = 8.0

        self.in_tunnel = False
        self.latest_fix = None
        self.latest_pose = None  # From /gazebo/model_states

        self.prev_clean_fix = None
        self.prev_noisy_fix = None

        rospy.Subscriber("/navsat/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_state_callback)

        self.pub = rospy.Publisher("/filtered_navsat/fix", NavSatFix, queue_size=10)

        rospy.Timer(rospy.Duration(0.5), self.check_tunnel_status)

        rospy.loginfo("GPS Filter Node using model_states started.")
        rospy.spin()

    def gps_callback(self, msg):
        if self.prev_clean_fix is None:
            # First message, initialize
            self.prev_clean_fix = msg
            self.pub.publish(msg)
            return

        if not self.in_tunnel:
            self.prev_clean_fix = msg
            self.pub.publish(msg)
        else:
            # # Compute delta from last clean fix
            # d_lat = msg.latitude - self.prev_clean_fix.latitude
            # d_lon = msg.longitude - self.prev_clean_fix.longitude
            # d_alt = msg.altitude - self.prev_clean_fix.altitude

            # # Add noise to the delta
            # noisy_d_lat = d_lat + random.uniform(-0.000001, 0.000001) 
            # noisy_d_lon = d_lon + random.uniform(-0.000001, 0.000001)
            # noisy_d_alt = d_alt + random.uniform(-0.000005, 0.000005)

            # In tunnel: republish last clean fix with modified covariance
            noisy_fix = NavSatFix()
            noisy_fix.header = msg.header
            noisy_fix.status = self.prev_clean_fix.status
            noisy_fix.latitude = self.prev_clean_fix.latitude 
            noisy_fix.longitude = self.prev_clean_fix.longitude 
            noisy_fix.altitude = self.prev_clean_fix.altitude
            noisy_fix.position_covariance = [	0.0001, 0, 0,      
    						0, 0.0001, 0,      
    						0, 0, 0.001   ]
            noisy_fix.position_covariance_type = self.prev_clean_fix.position_covariance_type

            self.pub.publish(noisy_fix)

            # # Update for next step
            # self.prev_clean_fix = msg
            # self.prev_noisy_fix = noisy_fix

            rospy.logwarn_throttle(5, "🚨 GPS in tunnel — publishing smoothed noisy GPS")
            # self.latest_fix = msg

    def model_state_callback(self, msg):
        try:
            idx = msg.name.index("UGV_Unimelb")
            self.latest_pose = msg.pose[idx]
        except ValueError:
            rospy.logwarn_throttle(5, "UGV_Unimelb not found in /gazebo/model_states")

    def check_tunnel_status(self, event):
        if self.latest_pose is None:
            rospy.logwarn_throttle(5, "No pose data yet")
            return

        x = self.latest_pose.position.x
        y = self.latest_pose.position.y

        in_tunnel_now = (
            self.tunnel_x_min <= x <= self.tunnel_x_max and
            self.tunnel_y_min <= y <= self.tunnel_y_max
        )

        if in_tunnel_now and not self.in_tunnel:
            rospy.logwarn("🚨 ENTERED tunnel zone — switching to noisy GPS")
        elif not in_tunnel_now and self.in_tunnel:
            rospy.loginfo("✅ EXITED tunnel zone — resuming clean GPS")

        self.in_tunnel = in_tunnel_now


if __name__ == "__main__":
    try:
        GPSFilterNode()
    except rospy.ROSInterruptException:
        pass

