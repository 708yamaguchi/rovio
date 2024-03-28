#!/usr/bin/env python
import rospy
import rosparam
import numpy as np
from sensor_msgs.msg import Imu

class IMUFilterNode:
    def __init__(self):
        # ノードの初期化
        rospy.init_node('imu_filter_node')

        # パブリッシャーとサブスクライバーの設定
        self.imu_pub = rospy.Publisher('imu_filtered', Imu, queue_size=10)
        self.imu_sub = rospy.Subscriber('imu', Imu, self.imu_callback)

        # フィルタリングのための変数初期化
        # ローパスフィルタの係数、必要に応じて調整。1にしたらフィルタなしと同じ
        rosparam.get_param("alpha", 0.1)
        self.last_filtered_values = {}

    def low_pass_filter(self, current_value, key):
        """
        ローパスフィルタを適用します。
        """
        if key not in self.last_filtered_values:
            self.last_filtered_values[key] = current_value
            return current_value

        filtered_value = self.alpha * current_value + (1 - self.alpha) * self.last_filtered_values[key]
        self.last_filtered_values[key] = filtered_value
        return filtered_value

    def imu_callback(self, msg):
        """
        IMUデータを受け取ったときのコールバック関数。
        """
        # IMUデータにフィルタを適用
        msg.linear_acceleration.x = self.low_pass_filter(msg.linear_acceleration.x, 'x')
        msg.linear_acceleration.y = self.low_pass_filter(msg.linear_acceleration.y, 'y')
        msg.linear_acceleration.z = self.low_pass_filter(msg.linear_acceleration.z, 'z')

        # フィルタリングされたデータをパブリッシュ
        self.imu_pub.publish(msg)

if __name__ == '__main__':
    node = IMUFilterNode()
    rospy.spin()
