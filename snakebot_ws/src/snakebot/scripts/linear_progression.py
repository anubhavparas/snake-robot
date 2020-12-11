#!/usr/bin/env python
import rospy

from std_msgs.msg import Float64

import sys, select, termios, tty
import math
import numpy as np
from gait import Gait

sin = math.sin
rad = math.radians


class LinearProgessionGait(Gait):
    def __init__(self, publishers, num_joints=10):
        self.num_joints = num_joints
        self.amplitudes, self.omegas, self.phases, self.axis_lags = self.get_angle_params(num_joints)
        self.publishers = publishers

    def get_angle_params(self, num_joints):
        amp_reduction_factor_hor = 0.6 #1.5 #0.4
        amp_reduction_factor_ver = 0.6 #1.5 #0.1

        a_hor = rad(0) * amp_reduction_factor_hor
        a_ver = rad(60) * amp_reduction_factor_ver
        amplitudes = [a_hor, a_ver]

        w_hor = rad(150)
        w_ver = rad(150)
        omegas = [w_hor, w_ver]

        ph_hor = rad(0)
        ph_ver = rad(120)
        phases = [ph_hor, ph_ver]

        axis_lag_hor = rad(0)
        axis_lag_ver = rad(0)
        axis_lags = [axis_lag_hor, axis_lag_ver]

        return amplitudes, omegas, phases, axis_lags


    def update_and_publish_angles(self, t):
        
        for i in range(self.num_joints):
            n = i+1
            a = self.amplitudes[n % 2]
            w = self.omegas[n % 2]
            ph = self.phases[n % 2]
            lag = self.axis_lags[n % 2]

            link_num = i // 2   # to calculate the ith horizontal/vertical link number

            angle = (a * sin(w*t + ph*n + lag))*math.pow(-1, link_num)
            rospy.loginfo('Angle = {} for joint = {}'.format(angle, i+1))
            self.publishers[i].publish(angle)


    
def get_publisher_nodes(num_joints):
    joint_publishers = []
    for i in range(num_joints):
        pub_node_name = '/snakebot/joint_{}_position_controller/command'.format(i+1)
        pub_node = rospy.Publisher(pub_node_name, Float64, queue_size=1000)
        joint_publishers.append(pub_node)

    return joint_publishers


if __name__=="__main__":
    
    rospy.init_node('snakebot_linear_progression', anonymous=True)
    
    num_joints = 10
    joint_publishers = get_publisher_nodes(num_joints=num_joints)
    gait = LinearProgessionGait(publishers=joint_publishers, num_joints=num_joints)


    rate = rospy.Rate(10)
    time_begin = rospy.Time.now()
    while not rospy.is_shutdown():
        time = (rospy.Time.now() - time_begin).to_sec()
        gait.update_and_publish_angles(t=time)

        #rospy.spin()
        rate.sleep()





    