#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import math

import cv2
import numpy as np
import os

# 全局变量用于存储机器人的姿态
robot_x = 0.0
robot_y = 0.0
robot_yaw = 0.0

# 全局变量用于存储积分误差和上一次误差
integral_error = 0.0
previous_error = 0.0

# 全局变量用于存储激光雷达数据
scan_data = []
# 全局变量用于表示是否已经接收到激光雷达数据
received_laser_data = True

def odom_callback(msg):
    global robot_x, robot_y, robot_yaw
    # 从里程计消息中提取位置和方向信息
    robot_x = msg.pose.pose.position.x
    robot_y = msg.pose.pose.position.y
    orientation_quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    )
    # 将四元数转换为欧拉角
    (_, _, robot_yaw) = euler_from_quaternion(orientation_quaternion)
    

# 在 laser_callback 函数中处理激光雷达数据
def laser_callback(msg):
    global scan_data, received_laser_data
    # 移除 'inf' 值，并确保至少存在一个有效的激光雷达距离值
    scan_data = [val for val in msg.ranges if 0.1 <= val <= 10.0]
    received_laser_data = True

def move_to_goal(goal_x, goal_y, use_curve):
    global robot_x, robot_y, robot_yaw, integral_error, previous_error, scan_data, received_laser_data

    rospy.init_node('move_to_goal', anonymous=False)
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    # 订阅里程计和激光雷达话题
    rospy.Subscriber('odom', Odometry, odom_callback)
    rospy.Subscriber('/scan', LaserScan, laser_callback)

    twist = Twist()
    rospy.loginfo('开始移动机器人到目标点')

    # PID参数
    kp = 3  # 调整比例项系数
    ki = 6  # 调整积分项系数
    kd = 6  # 调整微分项系数

    # 积分项的限制范围
    integral_limit = 1

    # 曲率因子
    curvature_factor = 1  # 调整曲率因子
    


# 获取图像文件的绝对路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    image_path = os.path.join(script_dir, 'ma.png')

# 读取图像
    image = cv2.imread(image_path)
    if image is None:
        rospy.loginfo("Failed to read image.")
        exit()

    rospy.loginfo("Image shape: %s" % str(image.shape))


# 将图像转换为灰度图像
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# 进行图像平滑处理
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

# 进行边缘检测
    edges = cv2.Canny(blurred, 50, 150)

# 进行轮廓提取
    _, contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    rospy.loginfo("Number of contours found: %d" % len(contours))

    if len(contours) > 0:
    # 找到最大的轮廓
        max_contour = max(contours, key=cv2.contourArea)
        rospy.loginfo("Max contour area: %f" % cv2.contourArea(max_contour))

	# 获取第一个轮廓的坐标点
    
    #points = max_contour[:, 0, :]
    #print(points)

    points = []

# 输出每个轮廓上的像素点坐标

    for point in max_contour:

        x, y = point[0]
        points.append([x, y])
    print(points)
    #suoxiao
    points = [[x / 100.0 for x in sublist] for sublist in points]
    #pingyi
    first_element = points[0]
    waypoints = [[x[0]-first_element[0], x[1]-first_element[1]] for x in points]
    
    print(waypoints)



    for goal in waypoints:
        goal_x, goal_y = goal
        # 重置激光雷达数据标志
        received_laser_data = False

        while not rospy.is_shutdown():
            # 计算目标点与当前机器人位置的方向
            angle_to_goal = math.atan2(goal_y - robot_y, goal_x - robot_x)

            # 计算角度差
            angle_difference = angle_to_goal - robot_yaw

            # 限制角度差在[-pi, pi]范围内
            while angle_difference > math.pi:
                angle_difference -= 2 * math.pi
            while angle_difference < -math.pi:
                angle_difference += 2 * math.pi

            # PID控制
            proportional_error = angle_difference
            integral_error += angle_difference
            derivative_error = angle_difference - previous_error


            # 限制积分项
            if integral_error > integral_limit:
                integral_error = integral_limit
            elif integral_error < -integral_limit:
                integral_error = -integral_limit

            # 计算调整角速度
            angular_speed = kp * proportional_error + ki * integral_error + kd * derivative_error

            # 根据曲率因子调整角速度
            if use_curve:
                angular_speed *= curvature_factor

            # 设置角速度，使机器人沿着规定的弧度前进
            twist.angular.z = angular_speed

            # 设置线速度，可以根据需要调整
            twist.linear.x = 0.2 if not received_laser_data or min(scan_data) >= 0.5 else 0.0

            # 发布速度命令
            cmd_pub.publish(twist)
	    #print(1)
            # 检测前方是否有障碍物
            if received_laser_data and min(scan_data) < 0.1:  # 适当调整阈值
	        #print(4)
                rospy.loginfo('前方有障碍物，停止机器人')
                rospy.sleep(1)  # 为避免立即重新启动，等待一段时间
            else:
                # 判断是否到达目标点
                distance_to_goal = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
	        #print(2)
                if distance_to_goal < 0.1:  # 适当调整容忍距离
	            #print(5)
                    rospy.loginfo('到达目标点')
                    break

            # 更新上一次误差
            previous_error = angle_difference

            rate.sleep()

    # 停止机器人
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    cmd_pub.publish(twist)

    # 等待一段时间，确保机器人停稳
    rospy.sleep(1)

if __name__ == '__main__':
    try:
        # 设置是否使用曲线的标志
        use_curve = False  # 设置为True表示走曲线，False表示走直线

        # 等待激光雷达数据
        while not received_laser_data and not rospy.is_shutdown():
            rospy.loginfo('等待激光雷达数据...')
            rospy.sleep(1)

        move_to_goal(0, 0, use_curve)
    except rospy.ROSInternalException:
        pass

