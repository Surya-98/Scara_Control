#!/usr/bin/env python
from frwd_kin.srv import plot, plotResponse
from frwd_kin.srv import control, controlResponse
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import numpy as np
import math
import time
import matplotlib.pyplot as plt
from controller_manager_msgs.srv import SwitchController
time_start = 0

np.set_printoptions(suppress=True)
i = 0
r = 0
ref = [0, 0, 0]
pub3 = rospy.Publisher(
    '/custom_scara/joint3_position_controller/command', Float64, queue_size=10)
pub2 = rospy.Publisher(
    '/custom_scara/joint2_position_controller/command', Float64, queue_size=10)
pub1 = rospy.Publisher(
    '/custom_scara/joint1_position_controller/command', Float64, queue_size=10)

k = 0


def callback1(data):
    global i
    global time_start
    global r
    global k
    if(i == 1):
        if(r == 1):
            file1 = open(
                "/home/killswitch/catkin_ws/src/scara/frwd_kin/scripts/myfile.txt", "w")
            file1.write(
                str(data.position[0])+' ' + str(k)+'\n')
            file1.close()
            r = 0
        else:
            file1 = open(
                "/home/killswitch/catkin_ws/src/scara/frwd_kin/scripts/myfile.txt", "a")
            file1.write(
                str(data.position[0])+' ' + str(k)+'\n')
            file1.close()
        if(rospy.get_rostime().secs-time_start > 10):
            file1 = open(
                "/home/killswitch/catkin_ws/src/scara/frwd_kin/scripts/myfile.txt", "r")
            y = file1.readlines()
            x1 = []
            y1 = []
            z1 = []
            for j in range(len(y)):
                temp = y[j].split()
                y1.append(float(temp[0]))
                x1.append(float(temp[1])*10.0/k)
                z1.append(ref)
            # print(x1, y1)
            i = 0
            k = 0
            # plt.plot(x1, y1, x1, z1)
            # plt.show(block=False)
            # time.sleep(10)
            # plt.close('all')
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
        ret = switch_controller(['joint1_position_controller', 'joint2_position_controller', 'joint3_position_controller'],
                                ['joint1_velocity_controller', 'joint2_velocity_controller', 'joint3_velocity_controller'], 2)
    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)

    global i
    global r
    global ref
    ref = data.q
    r = 1
    i = 1
    pub1.publish(ref[0])
    pub2.publish(ref[1])
    pub3.publish(ref[2])
    return controlResponse()


def control_server():

    rospy.init_node('control_p_server')

    s = rospy.Service('controlp', control, callback)
    rospy.Subscriber('/custom_scara/joint_states', JointState, callback1)
    rospy.spin()


if __name__ == "__main__":
    control_server()
