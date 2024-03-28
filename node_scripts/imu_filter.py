#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Imu

# Mainly Copied from https://www.utsbox.com/?page_id=523

class IMUFilterNode:
    def __init__(self):
        """
        The biquadratic (BiQuad) filter is a digital filter that calculates
        the output signal using the previous two input signals and
        the previous two output signals for the input signal.
        """
        # Input: raw IMU signal. Output: filtered IMU signal.
        self.imu_pub = rospy.Publisher('imu_filtered', Imu, queue_size=10)
        self.imu_sub = rospy.Subscriber('imu', Imu, self.imu_callback)
        # Store previous two inputs and outputs
        self.last1_inputs = {}
        self.last2_inputs = {}
        self.last1_outputs = {}
        self.last2_outputs = {}

        # Set filter parameter
        self.sampling_frequency = rospy.get_param(
            "~sampling_frequency", 100)
        self.cutoff_frequency = rospy.get_param(
            "~cutoff_frequency", 82)
        self.cutoff_bandwidth = rospy.get_param(
            "~cutoff_bandwidth", 0.3)
        self.q = rospy.get_param("~q", 1.0)
        filter_type = rospy.get_param(
            "~filter_type", "band_pass")
        if filter_type == "band_pass":
            omega = 2.0 * np.pi * self.cutoff_frequency / self.sampling_frequency
            alpha = np.sin(omega) * np.sinh(np.log(2.0) / 2.0 * self.cutoff_bandwidth * omega / sin(omega))
            self.a0 = 1.0 + alpha
            self.a1 = -2.0 * np.cos(omega)
            self.a2 = 1.0 - alpha
            self.b0 = alpha
            self.b1 = 0.0
            self.b2 = -1 * alpha
        elif filter_type == "low_pass":
            omega = 2.0 * np.pi * self.cutoff_frequency / self.sampling_frequency
            alpha = np.sin(omega) / (2.0 * self.q)
            self.a0 = 1.0 + alpha
            self.a1 = -2.0 * np.cos(omega)
            self.a2 = 1.0 - alpha
            self.b0 = (1.0 - np.cos(omega)) / 2.0
            self.b1 = 1.0 - np.cos(omega)
            self.b2 = (1.0 - np.cos(omega)) / 2.0

    def apply_filter(current_input, key):
        # Calculate filtered IMU singal
        current_output = self.b0 / self.a0 * current_input + self.b1 / self.a0 * self.last1_inputs[key] + self.b2 / self.a0 * self.last2_inputs[key] - self.a1/self.a0 * self.last1_outputs[key] - self.a1/self.a0 * self.last2_outputs[key]
        # Update inputs and outputs
        self.last2_inputs[key] = self.last1_inputs[key]
        self.last1_inputs[key] = current_input
        self.last2_outputs[key] = self.last1_outputs[key]
        self.last1_outputs[key] = current_output
        return current_output

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
