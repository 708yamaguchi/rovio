#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Imu

class IMUFilterNode:
    def __init__(self):
        self.imu_pub = rospy.Publisher('imu_filtered', Imu, queue_size=10)
        self.imu_sub = rospy.Subscriber('imu', Imu, self.imu_callback)

        # Parameter for Low pass filter. 1 means no filter.
        self.alpha = rospy.get_param("~alpha", 0.1)
        self.last_filtered_values = {}

    def low_pass_filter(self, current_value, key):
        """
        Apply low pass filter
        """
        if key not in self.last_filtered_values:
            self.last_filtered_values[key] = current_value
            return current_value

        filtered_value = self.alpha * current_value + (1 - self.alpha) * self.last_filtered_values[key]
        self.last_filtered_values[key] = filtered_value
        return filtered_value

    def imu_callback(self, msg):
        """
        Callback function for IMU message
        """
        # Apply filter to IMU data
        msg.orientation.x = self.low_pass_filter(msg.orientation.x, 'orientation_x')
        msg.orientation.y = self.low_pass_filter(msg.orientation.y, 'orientation_y')
        msg.orientation.z = self.low_pass_filter(msg.orientation.z, 'orientation_z')
        msg.orientation.w = self.low_pass_filter(msg.orientation.w, 'orientation_w')

        msg.linear_acceleration.x = self.low_pass_filter(msg.linear_acceleration.x, 'accel_x')
        msg.linear_acceleration.y = self.low_pass_filter(msg.linear_acceleration.y, 'accel_y')
        msg.linear_acceleration.z = self.low_pass_filter(msg.linear_acceleration.z, 'accel_z')

        msg.angular_velocity.x = self.low_pass_filter(msg.angular_velocity.x, 'gyro_x')
        msg.angular_velocity.y = self.low_pass_filter(msg.angular_velocity.y, 'gyro_y')
        msg.angular_velocity.z = self.low_pass_filter(msg.angular_velocity.z, 'gyro_z')

        # Publish filtered data
        self.imu_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('imu_filter_node')
    node = IMUFilterNode()
    rospy.spin()
