#!/usr/bin/env python

from frwd_kin.srv import inv, invResponse
import rospy
from frwd_kin.msg import q
import numpy as np
import math
np.set_printoptions(suppress=True)


def callback(data):
    l = 0.2
    m = 0.1
    pos = data.T
    print "Paramenters received were: \n ", pos, ' \n'
    x = pos[0]
    y = pos[1]
    z = pos[2]
    total = pos[5]
    q_return = q()
    D = ((math.pow(x, 2) + math.pow(y, 2))-0.08)/0.08
    q21 = math.degrees(math.atan2(math.sqrt(1-math.pow(D, 2)), D))
    q22 = math.degrees(math.atan2(-math.sqrt(1-math.pow(D, 2)), D))
    q11 = math.degrees(math.atan2(
        y, x)-math.atan2(0.2*math.sin(math.radians(q21)), (0.2+(0.2*math.cos(math.radians(q21))))))
    q12 = math.degrees(math.atan2(
        y, x)-math.atan2(0.2*math.sin(math.radians(q22)), (0.2+(0.2*math.cos(math.radians(q22))))))
    a1 = np.mod(q11+q21, 360) - np.mod(total, 360)
    a2 = np.mod(q12+q22, 360) - np.mod(total, 360)

    if(abs(a1) > abs(a2)):
        q_return.q1 = math.radians(q12)
        q_return.q2 = math.radians(q22)
    else:
        q_return.q1 = math.radians(q11)
        q_return.q2 = math.radians(q21)
    q_return.q3 = 0.1 - z
    print "Joint States are: \n", q_return, ' \n'
    return invResponse(q_return)


def inv_server():
    rospy.init_node('inv_server')
    s = rospy.Service('inv', inv, callback)
    rospy.spin()


if __name__ == "__main__":
    inv_server()
