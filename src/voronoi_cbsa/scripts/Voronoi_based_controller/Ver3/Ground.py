#!/usr/bin/env python3

import rospy
from voronoi_cbsa.msg import TargetInfoArray, TargetInfo
from geometry_msgs.msg import PoseStamped
from state_estimation.msg import EIFpairStamped
import pygame
import numpy as np
from time import time, sleep
from control import PTZCamera
from math import cos, acos, sqrt, exp, sin
from scipy.stats import multivariate_normal
import itertools

target_positions = {}
target_covariances = {}

target_positions[0] = [0.0, 0.0]
target_covariances[0] = [1, 0, 0, 1] # x, y 2D covariance

def norm(arr):
    sum = 0
    for i in range(len(arr)):
        sum += arr[i]**2

    return sqrt(sum)

def TargetPosCallback(msg):
    global target_positions
    position = np.array((msg.pose.position.x, msg.pose.position.y))
    target_positions[0] = position

def TargetCovCallback(msg):
    global target_covariances
    target_covariance = np.array(msg.P_hat).reshape((6, 6))
    target_covariances[0] = (target_covariance[:2, :2]).flatten()

def TargetDynamics(x, y, v):
    spd = 0.005
    turn = np.random.randint(-50, 50)/1800*np.pi
    rot = np.array([[cos(turn), -sin(turn)],
                    [sin(turn), cos(turn)]])
    v = rot@v.reshape(2,1)
    vx = v[0] if v[0]*spd + x > 0 and v[0]*spd + x < 24 else -v[0]
    vy = v[1] if v[1]*spd + y > 0 and v[1]*spd + y < 24 else -v[1]
    
    return (x,y), np.asarray([[0],[0]])
    #return (np.round(float(np.clip(v[0]*spd + x, 0, 24)),3), np.round(float(np.clip(v[1]*spd + y, 0, 24)),3)),\
    #         np.round(np.array([[vx],[vy]]), len(str(spd).split(".")[1]))

def RandomUnitVector():
    v = np.asarray([np.random.normal() for i in range(2)])
    return v/norm(v)

if __name__ == "__main__":
    rospy.init_node('ground_control_station', anonymous=True, disable_signals=True)
    rate = rospy.Rate(60)

    target_pos_sub = rospy.Subscriber("/iris_1/THEIF/pose", PoseStamped, callback = TargetPosCallback)
    target_cov_sub = rospy.Subscriber("/iris_1/TEIF/fusionPairs", EIFpairStamped, callback = TargetCovCallback)

    target_pub = rospy.Publisher("/target", TargetInfoArray, queue_size=10)
    
    # 若沒有設定targets位置(pos=(0,0) ), 則隨機給x, y的位置
    def random_pos(pos=(0,0)):
        if pos == (0,0):
            x = np.random.random()*15 + 5
            y = np.random.random()*15 + 5
            return np.array((x,y))
        else:
            return np.asarray(pos)
    # # target 1 設定在(18,5), target 2 設定在(18,18), 1是standard deviation, 10是weights
    # targets = [[random_pos((18,5)), 1, 10, RandomUnitVector(), ['camera', 'manipulator']],
    #            [random_pos((18,18)), 1, 10, RandomUnitVector(), ['camera', 'smoke_detector']]]
    targets = [[target_positions[0], target_covariances[0], 10, RandomUnitVector(), ['camera']]]
    
    while not rospy.is_shutdown():
            
        grid_size = rospy.get_param("/grid_size", 0.1)
        tmp = []

        for i in range(len(targets)):
            pos, vel = TargetDynamics(targets[i][0][0], targets[i][0][1], targets[i][3])
            targets[i][0] = pos
            targets[i][3] = vel

            target_msg = TargetInfo()
            target_msg.id = i
            target_msg.position.x = pos[0]
            target_msg.position.y = pos[1]
            target_msg.covariance = targets[i][1]
            target_msg.weight = targets[i][2]
            target_msg.velocity.linear.x = vel[0]
            target_msg.velocity.linear.y = vel[1]
            target_msg.required_sensor = targets[i][4]

            tmp.append(target_msg)

        targets_array = TargetInfoArray()
        targets_array.targets = [tmp[i] for i in range(len(tmp))]

        target_pub.publish(targets_array)
        rate.sleep()