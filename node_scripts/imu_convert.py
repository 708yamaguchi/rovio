#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu


def cb(msg):
    imu_converted = msg
    imu_converted.angular_velocity.x = -1 * imu_converted.angular_velocity.x
    imu_converted.angular_velocity.y = -1 * imu_converted.angular_velocity.y
    imu_converted.angular_velocity.z = -1 * imu_converted.angular_velocity.z
    imu_converted.linear_acceleration.x = -1 * imu_converted.linear_acceleration.x
    imu_converted.linear_acceleration.y = -1 * imu_converted.linear_acceleration.y
    imu_converted.linear_acceleration.z = -1 * imu_converted.linear_acceleration.z
    pub.publish(imu_converted)


rospy.init_node('imu_convert')
rospy.Subscriber('imu', Imu, cb)
pub = rospy.Publisher('imu_converted', Imu, queue_size=100)
rospy.spin()
