#!/usr/bin/env python

from frwd_kin.srv import frwd, frwdResponse
import rospy
from frwd_kin.msg import q
import numpy as np
import math
np.set_printoptions(suppress=True)


def rotationMatrixToEulerAngles(R):
    # zyx euler
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([math.degrees(x), math.degrees(y), math.degrees(z)])


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
    K = T[0]
    for i in range(1, 3):
        K = np.matmul(K, T[i])
    E = rotationMatrixToEulerAngles(K[0:3, 0:3])
    P = K[0:3, 3]
    print "Paramenters received were: \n", [data.qservice.q1, data.qservice.q2, data.qservice.q3], ' \n'
    print "End effector's position is: \n", K[0:3, 3], ' \n'
    print "Euler is: \n", E, ' \n'
    K1 = np.append(P, E)
    return frwdResponse(K1)


def frwd_server():
    rospy.init_node('frwd_server')
    s = rospy.Service('frwd', frwd, callback)
    rospy.spin()


if __name__ == "__main__":
    frwd_server()
