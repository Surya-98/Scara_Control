#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import numpy as np
import math
from frwd_kin.msg import q
import sys
from frwd_kin.srv import *
import math
np.set_printoptions(suppress=True)


def f_kin(q_now):
    inp = q()
    inp.q1 = math.degrees(q_now[0])
    inp.q2 = math.degrees(q_now[1])
    inp.q3 = q_now[2]

    rospy.wait_for_service('frwd')
    try:
        forward = rospy.ServiceProxy('frwd', frwd)
        resp1 = forward(inp)
        return resp1.T
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def i_kin(T_now):
    inp = T_now
    rospy.wait_for_service('inv')
    try:
        inverse = rospy.ServiceProxy('inv', inv)
        resp1 = inverse(inp)
        return resp1.qservice
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def callback(data):
    print "The joint positions from Gazebo are : \n", data.position, "\n"
    T = f_kin(data.position)
    print "The output from Forward Kinematics is : \n", T, "\n"
    print "The output from Inverse Kinematics is : \n", i_kin(T), "\n"

    if(data):
        rospy.signal_shutdown("")


def listener():

    rospy.init_node('assignment_1', anonymous=True)
    rospy.Subscriber('/custom_scara/joint_states', JointState, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
