#!/usr/bin/env python

import rospy
from frwd_kin.msg import q


def talker():
    pub = rospy.Publisher('chatter', q, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        variable = q()
        variable.q1 = 0
        variable.q2 = 0
        variable.q3 = 0
        print(variable)
        pub.publish(variable)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
