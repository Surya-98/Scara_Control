#!/usr/bin/env python
import rospy
from frwd_kin.msg import q
import numpy as np
import math
np.set_printoptions(suppress=True)


def dhparam2mat(theta, d, alpha, a):
    theta = math.radians(theta)
    alpha = math.radians(alpha)
    Rz = np.array([[math.cos(theta), -math.sin(theta), 0, 0],
                   [math.sin(theta), math.cos(theta), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    Tz = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d], [0, 0, 0, 1]])

    Tx = np.array([[1, 0, 0, a], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    Rx = np.array([[1, 0, 0, 0], [0, math.cos(alpha), -math.sin(alpha), 0],
                   [0, math.sin(alpha), math.cos(alpha), 0], [0, 0, 0, 1]])

    T = np.matmul(Rz, Tz)
    T = np.matmul(T, Tx)
    T = np.matmul(T, Rx)

    return T


def callback(data):
    l = 10
    m = 20
    n = 30
    pos = np.array([0, 0, 0])
    theta = np.array([data.q1, data.q2, data.q3])
    d = np.array([l, 0, 0])
    alpha = np.array([-90, 0, 90])
    a = np.array([0, m, n])
    T = []
    for i in range(0, 3):
        T.append(dhparam2mat(theta[i], d[i], alpha[i], a[i]))
    K = T[0]
    for i in range(1, 3):
        K = np.matmul(K, T[i])
    print "Paramenters received were: ", [data.q1, data.q2, data.q3]
    print "End effector's position is:", K[0:3, 3]
    print "End effector's direction is:", ' \n', K[0:3, 0:3]


def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', q, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
