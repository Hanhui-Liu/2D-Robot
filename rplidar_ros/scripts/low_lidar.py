#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math

def scan_callback(data):
    # 进行降采样处理
    downsampled_data = downsample_scan(data)
    
    # 发布降采样后的数据到 /scan_low 话题
    scan_low_pub.publish(downsampled_data)

def downsample_scan(scan_data):
    # 获取原始扫描数据
    ranges = scan_data.ranges

    # 计算降采样因子
    downsample_factor = int(len(ranges) / 1000)

    # 使用最大值法进行降采样
    downsampled_ranges = [np.max(ranges[i:i+downsample_factor]) for i in range(0, len(ranges), downsample_factor)]

    # 创建降采样后的消息对象
    downsampled_scan_data = LaserScan()
    
    # 将降采样后的数据填充到消息对象中
    downsampled_scan_data.header = scan_data.header
    downsampled_scan_data.angle_min = scan_data.angle_min
    downsampled_scan_data.angle_max = scan_data.angle_max
    downsampled_scan_data.angle_increment = scan_data.angle_increment * downsample_factor
    downsampled_scan_data.time_increment = scan_data.time_increment * downsample_factor
    downsampled_scan_data.scan_time = scan_data.scan_time
    downsampled_scan_data.range_min = scan_data.range_min
    downsampled_scan_data.range_max = scan_data.range_max

    # 数据过滤
    filtered_ranges = []
    for i in range(len(downsampled_ranges)):
        angle = scan_data.angle_min + (i * downsampled_scan_data.angle_increment)
        distance = downsampled_ranges[i]

        # 将极坐标转换为笛卡尔坐标
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)

        # 判断点是否在矩形内
        if (0.733 >= x >= -0.05) and (0.25 >= y >= -0.25):
            filtered_ranges.append(float('nan'))
        else:
            filtered_ranges.append(distance)

    downsampled_scan_data.ranges = filtered_ranges


    return downsampled_scan_data

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('scan_downsampler')
    
    # 创建订阅器，订阅 /scan 话题，数据类型为 LaserScan
    scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
    
    # 创建发布器，发布降采样后的数据到 /scan_low 话题，数据类型为 LaserScan
    scan_low_pub = rospy.Publisher('/scan_low', LaserScan, queue_size=10)
    
    # 循环等待回调函数的触发
    rospy.spin()