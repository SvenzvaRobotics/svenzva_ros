#!/usr/bin/env python

"""
A [Joint, Torque] agnostic method of enabling torque mode across all joints on the arm

Author: Maxwell Svetlik
"""

import rospy
from std_msgs.msg import Bool
from dynamixel_controllers.srv import *

def pub():
    rospy.init_node('enable_torque_mode', anonymous=True)
    #val = TorqueEnableRequest()
    val = True
    j1 = rospy.ServiceProxy('/joint_1/torque_enable', TorqueEnable)
    j2 = rospy.ServiceProxy('/joint_2/torque_enable', TorqueEnable)
    j3 = rospy.ServiceProxy('/joint_3/torque_enable', TorqueEnable)
    j4 = rospy.ServiceProxy('/joint_4/torque_enable', TorqueEnable)
    j5 = rospy.ServiceProxy('/joint_5/torque_enable', TorqueEnable)
    j6 = rospy.ServiceProxy('/joint_6/torque_enable', TorqueEnable)
    resp1 = j1(val)
    resp2 = j2(val)
    resp3 = j3(val)
    resp4 = j4(val)
    resp5 = j5(val)
    resp6 = j6(val)


if __name__ == '__main__':
    try:
        pub()
    except rospy.ROSInterruptException:
        pass

