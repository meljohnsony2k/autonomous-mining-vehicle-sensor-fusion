#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class SensorCovarianceScaler:
    def __init__(self):
        rospy.init_node('sensor_covariance_scaler')

        # Publishers
        self.imu_pub = rospy.Publisher('/imu_scaled', Imu, queue_size=10)
        self.gps_pub = rospy.Publisher('/odometry/gps_scaled', Odometry, queue_size=10)
        self.amcl_pub = rospy.Publisher('/amcl_odom', Odometry, queue_size=10)

        # Subscribers
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        rospy.Subscriber('/odometry/gps', Odometry, self.gps_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)

        rospy.loginfo("SensorCovarianceScaler node started.")
        rospy.spin()

    def imu_callback(self, msg):
        # Set covariance for yaw orientation
        msg.orientation_covariance = [999999, 0, 0,
                                      0, 999999, 0,
                                      0, 0, 1]  # yaw only

        # Angular velocity (only Z used)
        msg.angular_velocity_covariance = [999999, 0, 0,
                                           0, 999999, 0,
                                           0, 0, 1]

        # Linear acceleration (X and Y only)
        msg.linear_acceleration_covariance = [1, 0, 0,
                                              0, 1, 0,
                                              0, 0, 999999]

        self.imu_pub.publish(msg)

    def gps_callback(self, msg):
        # Shift position
        msg.pose.pose.position.x += -3.0
        msg.pose.pose.position.y += -2.0

        # Read original GPS x-position covariance
        gps_cov = msg.pose.covariance[0]

        # Set low or high trust based on incoming covariance
        if gps_cov <= 1e-10:
            cov_xy = 0.0001  # High trust
        else:
            cov_xy = 100.0  # Low trust

        # Apply to pose covariance matrix
        msg.pose.covariance = [
            cov_xy, 0, 0, 0, 0, 0,
            0, cov_xy, 0, 0, 0, 0,
            0, 0, 999999, 0, 0, 0,
            0, 0, 0, 999999, 0, 0,
            0, 0, 0, 0, 999999, 0,
            0, 0, 0, 0, 0, 999999
        ]

        self.gps_pub.publish(msg)

    def amcl_callback(self, msg):
        odom = Odometry()
        odom.header = msg.header
        odom.pose = msg.pose
        odom.child_frame_id = "base_link"
        self.amcl_pub.publish(odom)

if __name__ == '__main__':
    try:
        SensorCovarianceScaler()
    except rospy.ROSInterruptException:
        pass
