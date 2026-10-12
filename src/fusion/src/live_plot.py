#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

def moving_average(data, window_size=10):
        return np.convolve(data, np.ones(window_size)/window_size, mode='same')

class LiveTrajectoryPlot:
    def __init__(self):
        self.gps_x, self.gps_y = [], []
        self.filtered_x, self.filtered_y = [], []
        self.amcl_x, self.amcl_y = [], []
        self.gt_x, self.gt_y = [], []

        self.smoothed_fx, self.smoothed_fy = [], []
        self.alpha = 0.1  # Smoothing factor (0 < alpha < 1), lower = smoother

        self.fused_msg_count = 0  # Counter for filtered messages

        # Subscribers
        rospy.Subscriber('/odometry/gps_scaled', Odometry, self.gps_callback)
        rospy.Subscriber('/odometry/filtered', Odometry, self.filtered_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        #rospy.Subscriber('/ground_truth/odometry/gps_gt', Odometry, self.gt_callback)

        # Plot setup
        self.fig, self.ax = plt.subplots()
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=200)

    def gps_callback(self, msg):
        self.fused_msg_count += 1
        if self.fused_msg_count <= 100:
            return  # Ignore first 10 messages
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if x is not None and y is not None:
            self.gps_x.append(x)
            self.gps_y.append(y)

    def filtered_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if x is not None and y is not None:
            if len(self.filtered_x) > len(self.filtered_y):
                self.filtered_x = self.filtered_x[:len(self.filtered_y)]
            elif len(self.filtered_y) > len(self.filtered_x):
                self.filtered_y = self.filtered_y[:len(self.filtered_x)]

        self.filtered_x.append(x)
        self.filtered_y.append(y)


    def amcl_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if x is not None and y is not None:
            self.amcl_x.append(x)
            self.amcl_y.append(y)


    '''def gt_callback(self, msg):
        self.gt_x.append(msg.pose.pose.position.x)
        self.gt_y.append(msg.pose.pose.position.y)'''


    def update_plot(self, frame):
        self.ax.clear()

        # Sync lengths for safe plotting
        min_len_gps = min(len(self.gps_x), len(self.gps_y))
        min_len_amcl = min(len(self.amcl_x), len(self.amcl_y))
        min_len_filtered = min(len(self.filtered_x), len(self.filtered_y))

        self.ax.plot(self.gps_x[:min_len_gps], self.gps_y[:min_len_gps], color='green', label='Noisy GPS')
        self.ax.plot(self.amcl_x[:min_len_amcl], self.amcl_y[:min_len_amcl], color='blue', label='AMCL Pose')
        #self.ax.plot(self.filtered_x[:min_len_filtered], self.filtered_y[:min_len_filtered], color='red', label='UKF Fused')
        # Apply exponential smoothing
        if len(self.filtered_x) >= 1:
            if not self.smoothed_fx:  # Initialize on first data
                self.smoothed_fx.append(self.filtered_x[0])
                self.smoothed_fy.append(self.filtered_y[0])
            else:
                new_x = self.alpha * self.filtered_x[-1] + (1 - self.alpha) * self.smoothed_fx[-1]
                new_y = self.alpha * self.filtered_y[-1] + (1 - self.alpha) * self.smoothed_fy[-1]
                self.smoothed_fx.append(new_x)
                self.smoothed_fy.append(new_y)

            self.ax.plot(self.smoothed_fx, self.smoothed_fy, color='red', label='Fused UKF')

        
        #self.ax.plot(self.gt_x, self.gt_y, color='purple', linestyle='--', label='Ground Truth GPS')

        self.ax.set_title('Live Trajectories: GPS, UKF, AMCL, Ground Truth')
        self.ax.set_xlabel('Position X (m)')
        self.ax.set_ylabel('Position Y (m)')
        self.ax.grid(True)
        self.ax.axis('equal')
        self.ax.legend()

if __name__ == '__main__':
    rospy.init_node('live_trajectory_plot', anonymous=True)
    plotter = LiveTrajectoryPlot()
    plt.show()
