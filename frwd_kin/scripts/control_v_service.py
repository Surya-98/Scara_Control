#!/usr/bin/env python
from controller_manager_msgs.srv import SwitchController
from frwd_kin.srv import frwd_v, frwd_vResponse
from frwd_kin.srv import inv_v, inv_vResponse
from frwd_kin.srv import plot, plotResponse
from frwd_kin.srv import controlv, controlvResponse
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import numpy as np
import math
import time
import matplotlib.pyplot as plt
time_start = 0

np.set_printoptions(suppress=True)
i = 0
r = 0
ref = [0, 0, 0]
pub3 = rospy.Publisher(
    '/custom_scara/joint3_velocity_controller/command', Float64, queue_size=10)
pub2 = rospy.Publisher(
    '/custom_scara/joint2_velocity_controller/command', Float64, queue_size=10)
pub1 = rospy.Publisher(
    '/custom_scara/joint1_velocity_controller/command', Float64, queue_size=10)

k = 0


def callback1(data):
    global i
    global time_start
    global r
    global k
    if(i == 1):
        # print(ref)

        inverse = rospy.ServiceProxy('inv_v', inv_v)
        t1 = list(data.position)
        t1.extend(list(ref))
        resp = inverse(tuple(t1))
        pub1.publish(resp.q_dot[0])
        pub2.publish(resp.q_dot[1])
        pub3.publish(resp.q_dot[2])

        forward = rospy.ServiceProxy('frwd_v', frwd_v)
        t2 = list(data.position)
        t2.extend(list(data.velocity))
        # print(t2)
        resp1 = forward(tuple(t2))
        if(r == 1):
            file1 = open(
                "/home/killswitch/catkin_ws/src/scara/frwd_kin/scripts/myfilev.txt", "w")
            file1.write(
                str(resp1.v[0])+' ' + str(resp1.v[1])+' ' + str(resp1.v[2])+' ' + str(k)+'\n')
            file1.close()
            r = 0
        else:
            file1 = open(
                "/home/killswitch/catkin_ws/src/scara/frwd_kin/scripts/myfilev.txt", "a")
            file1.write(
                str(resp1.v[0])+' ' + str(resp1.v[1])+' ' + str(resp1.v[2])+' ' + str(k)+'\n')
            file1.close()
        if(rospy.get_rostime().secs-time_start > 5):
            file1 = open(
                "/home/killswitch/catkin_ws/src/scara/frwd_kin/scripts/myfilev.txt", "r")
            y = file1.readlines()
            # print(y)
            x1 = []
            q1 = []
            q2 = []
            q3 = []
            z1 = [[], [], []]
            for j in range(len(y)):
                temp = y[j].split()

                # print("temp is:", temp)
                q1.append(float(temp[0]))
                q2.append(float(temp[1]))
                q3.append(float(temp[2]))
                x1.append(float(temp[3])*5.0/k)
                z1[0].append(ref[0])
                z1[1].append(ref[1])
                z1[2].append(ref[2])
            i = 0
            k = 0
            pub1.publish(0)
            pub2.publish(0)
            pub3.publish(0)
            plt.figure()
            plt.plot(x1, q1, x1, z1[0])
            plt.show(block=False)
            plt.figure()
            plt.plot(x1, q2, x1, z1[1])
            plt.show(block=False)
            plt.figure()
            plt.plot(x1, q3, x1, z1[2])
            plt.show(block=False)
            time.sleep(10)
            plt.close('all')
        k = k + 1


def callback(data):
    global time_start
    time_start = rospy.get_rostime().secs
    time_start_n = rospy.get_rostime().nsecs
    rospy.wait_for_service(
        '/custom_scara/controller_manager/switch_controller')
    try:
        switch_controller = rospy.ServiceProxy(
            '/custom_scara/controller_manager/switch_controller', SwitchController)
        ret = switch_controller(['joint1_velocity_controller', 'joint2_velocity_controller', 'joint3_velocity_controller'],
                                ['joint1_position_controller', 'joint2_position_controller', 'joint3_position_controller'], 2)
    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)

    global i
    global r
    global ref
    ref = data.q
    i = 1
    r = 1
    return controlvResponse()


def control_server():
    rospy.init_node('control_v_server')
    s = rospy.Service('controlv', controlv, callback)
    rospy.Subscriber('/custom_scara/joint_states', JointState, callback1)
    rospy.spin()


if __name__ == "__main__":
    control_server()
