#!/usr/bin/env python

from frwd_kin.srv import inv_v, inv_vResponse
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


def skew(vector):
    return np.array([[0, -vector[2], vector[1]],
                     [vector[2], 0, -vector[0]],
                     [-vector[1], vector[0], 0]])


def Jacobian(q1, q2, q3):
    l = 0.2
    m = 0.1
    pos = np.array([0, 0, 0])
    theta = np.array([q1, q2, 0])
    d = np.array([l, 0, q3 + m])
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
    J[0:3, 0] = np.matmul(skew(TT[0][0:3, 2]), TT[2][0:3, 3] - [0, 0, 0])
    J[0:3, 1] = np.matmul(skew(TT[1][0:3, 2]), TT[2][0:3, 3] - TT[0][0:3, 3])
    J[0:3, 2] = TT[2][0:3, 2]
    J[3:6, 0] = [0, 0, 1]
    J[3:6, 1] = TT[0][0:3, 2]
    J[3:6, 2] = [0, 0, 0]
    return J


def callback(data):
    values = data.v
    J = Jacobian(values[0], values[1], values[2])
    J = np.linalg.pinv(J)
    print(J)
    Qdot = np.matmul(J, [values[3], values[4], values[5],
                         values[6], values[7], values[8]])
    return inv_vResponse(Qdot)


def inv_v_server():
    rospy.init_node('inv_v_server')
    s = rospy.Service('inv_v', inv_v, callback)
    rospy.spin()


if __name__ == "__main__":
    inv_v_server()
