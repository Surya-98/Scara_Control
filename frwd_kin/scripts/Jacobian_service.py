#!/usr/bin/env python

from frwd_kin.srv import jacobian, jacobianResponse
import rospy
from frwd_kin.msg import q
import numpy as np
import math
np.set_printoptions(suppress=True)


def skew(vector):
    return np.array([[0, -vector[2], vector[1]],
                     [vector[2], 0, -vector[0]],
                     [-vector[1], vector[0], 0]])


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
    print data
    l = 0.2
    m = 0.1
    pos = np.array([0, 0, 0])
    theta = np.array([data.qservice.q1, data.qservice.q2, 0])
    d = np.array([l, 0, data.qservice.q3 + m])
    alpha = np.array([0, 180, 0])
    a = np.array([l, l, 0])
    T = []
    for i in range(0, 3):
        T.append(dhparam2mat(theta[i], d[i], alpha[i], a[i]))
    TT = []
    K = T[0]
    TT.append(K)
    for i in range(1, 3):
        K = np.matmul(K, T[i])
        TT.append(K)
    J = np.zeros([6, 3])
    J[0:3, 0] = np.matmul(skew([0, 0, 1]), TT[2][0:3, 3] - [0, 0, 0])
    J[0:3, 1] = np.matmul(skew(TT[0][0:3, 2]), TT[2][0:3, 3] - TT[0][0:3, 3])
    J[0:3, 2] = TT[2][0:3, 2]
    J[3:6, 0] = [0, 0, 1]
    J[3:6, 1] = TT[0][0:3, 2]
    J[3:6, 2] = [0, 0, 0]
    print "Paramenters received were: ", [data.qservice.q1, data.qservice.q2, data.qservice.q3]
    print "Jacobian is:", '\n', J
    print "TT=", TT
    J1 = np.ndarray.flatten(J)
    return jacobianResponse([3, 6], J1)


def jacobian_server():
    rospy.init_node('jacobian_server')
    s = rospy.Service('jacobian', jacobian, callback)
    rospy.spin()


if __name__ == "__main__":
    jacobian_server()
