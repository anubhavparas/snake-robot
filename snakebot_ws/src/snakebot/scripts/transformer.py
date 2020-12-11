#!/usr/bin/env python
import rospy

from std_msgs.msg import Float64

import sys, select, termios, tty
import math
import numpy as np
from gait import Gait

sin = math.sin
rad = math.radians
count = 1
TRANSIT_TIME = 4
class TransformerGait(Gait):
    def __init__(self, publishers, num_joints=10):
        self.num_joints = num_joints
        self.publishers = publishers
        self.key_point = 0


    def update_and_publish_angles(self, t):
        # key-frames - set of joint angles as per different time-stamps
        angle_matrix = [
            [90,  0,       -90,    0,    0,     0,  -90,    0,    90,      0],

            [120,  0,      -110,   0,   -30,    0,  -110,   0,    120,     0],
            [135,  0,      -120,   0,   -30,    0,  -120,   0,    135,     0],

            [135,  0,      -120,  -60,  -30,   -60, -120,   0,    135,     0],
            [135,  0,      -120,  -90,  -10,   -90, -120,   0,    135,     0],
            [135,  0,      -120,  -90,   0,    -90, -120,   0,    135,     0],

            ## biped is formed
            ## moving on to make it a closed polygon
            [125,  0,      -80,   -90,   0,    -90,  -80,   0,    125,     0],
            [125,  0,      -60,   -90,   0,    -90,  -60,   0,    125,     0],
            [125,  0,      -20,   -90,   0,    -90,  -20,   0,    125,     0],
            [120,  0,       20,   -90,   0,    -90,   20,   0,    120,     0],
            [120,  0,       40,   -90,   0,    -90,   40,   0,    120,     0],
            [120,  0,       60,   -90,   0,    -90,   60,   0,    120,     0],

            #renctangle shape done
            #over to making the legs more closer
            [120,  0,       60,    0,     0,    0,    60,   0,    120,     0],

            [90,  0,        90,    0,     0,    0,    90,   0,    90,      0],
            [85,  0,        95,    20,    0,   -20,   95,   0,    85,      0]
        ]

        global count  #initial value of count = 0
        if (t >= TRANSIT_TIME*count):  # transition time between two key-frames: t_transit = 4 sec
            count+=1
            self.key_point += 1
            self.key_point = min(len(angle_matrix)-1, self.key_point)
        angles = angle_matrix[self.key_point]
        
        for i in range(self.num_joints):
            angle = rad(angles[i])
            rospy.loginfo('Angle = {} for joint = {}'.format(angle, i+1))
            self.publishers[i].publish(angle)




    
def get_publisher_nodes(num_joints):
    joint_publishers = []
    for i in range(num_joints):
        pub_node_name = '/snakebot/joint_{}_position_controller/command'.format(i+1)
        pub_node = rospy.Publisher(pub_node_name, Float64, queue_size=10)
        joint_publishers.append(pub_node)

    return joint_publishers


if __name__=="__main__":
    
    rospy.init_node('snakebot_transformer_biped', anonymous=True)
    
    num_joints = 10
    joint_publishers = get_publisher_nodes(num_joints=num_joints)
    gait = TransformerGait(publishers=joint_publishers, num_joints=num_joints)


    rate = rospy.Rate(10)
    time_begin = rospy.Time.now()

    while not rospy.is_shutdown():
        time = (rospy.Time.now() - time_begin).to_sec()
        gait.update_and_publish_angles(time)
        rate.sleep()





    