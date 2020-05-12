#!/usr/bin/env python
#! -*- coding:utf-8 -*-

import rospy
import message_filters
from nav_msgs.msg import Odometry
from diff_msgs.msg import WheelSpeedStatus
import matplotlib.pyplot as plt
import csv

# 全局变量
store_path = '/home/flztiii/Documents/turtlebot_test/compare/'
index = 0
distance = 0.0
distances = []
velocitys = []
last_odom = []

# 消息回调函数
def callback(odom_msg, diff_msg):
    velocity = (diff_msg.leftWheelSpeed + diff_msg.rightWheelSpeed) / 2000.0
    # 判断是不是第一个odom
    if len(last_odom) == 0:
        last_odom.append(odom_msg)
        distances.append(distance)
        velocitys.append(velocity)
    else:
        # 如果不是第一个
        distance += sqrt((odom_msg.pose.pose.position.x - last_odom[0].pose.pose.position.x) ** 2 + (odom_msg.pose.pose.position.y - last_odom[0].pose.pose.position.y) ** 2)
        last_odom[0] = odom_msg
        distances.append(distance)
        velocitys.append(velocity)
    index += 1
    if index % 100 == 0:
        fig = plt.figure(figsize=(14, 14))
        plt.plot(distances, velocitys)
        plt.savefig(store_path + 'vd.png')
        # 保存原本数据为csv
        with open(store_path + 'vd.csv', 'w') as f:
            writer = csv.writer(f)
            for i,_ in enumerate(distances):
                writer.writerow([distances[i], velocitys[i]])

# 主函数
def main():
    # 初始化节点
    rospy.init_node('compare_velocity_profile_node', anonymous=True)
    # 订阅速度和定位信息
    odom_topic = '/planning_input/localization'
    vel_topic = '/wheel_speed_status'

    odom_sub = message_filters.Subscriber(odom_topic, Odometry)
    vel_sub = message_filters.Subscriber(vel_topic, WheelSpeedStatus)

    ts = message_filters.TimeSynchronizer([odom_sub, vel_sub], 10)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == "__main__":
    main()