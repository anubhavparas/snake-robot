#!/usr/bin/env python
import rospy

from std_msgs.msg import Float64

import sys, select, termios, tty
import math
from gait import Gait

sin = math.sin
rad = math.radians

class ForwardGait(Gait):
    def __init__(self, publishers, num_joints=10):
        self.num_joints = num_joints
        self.amplitudes, self.omegas, self.phases = self.get_angle_params(num_joints)
        self.publishers = publishers

    def get_angle_params(self, num_joints):
        # A0=0;A1=90;A2=30;A3=0;A4=30;A5=0;A6=30;A7=0;A8=30;A9=90;
        # W0=90;W1=0;W2=3;W3=1;W4=3;W5=1;W6=3;W7=1;W8=3;W9=1;
        # ph0=0;ph1=180;ph2=0;ph3=180;ph4=120;ph5=0;ph6=240;ph7=180;ph8=360;ph9=0;
        
        amplitudes = [0, 90, 30, 0, 30, 0, 30, 0, 30, 90]
        omegas = [90, 0, 3, 1, 3, 1, 3, 1, 3, 1]
        phases = [0, 180, 0, 180, 120, 0, 240, 180, 360, 0]

        return amplitudes, omegas, phases


    def update_and_publish_angles(self, t):
        
        for i in range(self.num_joints):
            a = rad(self.amplitudes[i])
            w = self.omegas[i]
            ph = rad(self.phases[i])

            angle = a*sin(w*t + ph)
            rospy.loginfo('Angle = {} for joint = {}'.format(angle, i+1))
            self.publishers[i].publish(angle)




    
def get_publisher_nodes(num_joints):
    joint_publishers = []
    for i in range(num_joints):
        pub_node_name = '/snakebot/joint_{}_position_controller/command'.format(i)
        print(pub_node_name)
        pub_node = rospy.Publisher(pub_node_name, Float64, queue_size=10)
        joint_publishers.append(pub_node)

    return joint_publishers


if __name__=="__main__":
    
    rospy.init_node('snakebot_forward', anonymous=True)
    num_joints = 10
    joint_publishers = get_publisher_nodes(num_joints=num_joints)
    gait = ForwardGait(publishers=joint_publishers, num_joints=num_joints)


    rate = rospy.Rate(8)
    time_begin = rospy.Time.now()
    while not rospy.is_shutdown():
        time = (rospy.Time.now() - time_begin).to_sec()
        gait.update_and_publish_angles(t=time)

        #rospy.spin()
        rate.sleep()





    