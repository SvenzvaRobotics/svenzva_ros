#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState


q0_loc = Float64()
q1_loc = Float64()
q2_loc = Float64()
q3_loc = Float64()
q4_loc = Float64()
q5_loc = Float64()
f_loc = Float64()

def cb_q0(data):
    global q0_loc
    q0_loc = data.current_pos

def cb_q1(data):
    global q1_loc
    q1_loc = data.current_pos

def cb_q2(data):
    global q2_loc
    q2_loc = data.current_pos

def cb_q3(data):
    global q3_loc
    q3_loc = data.current_pos

def cb_q4(data):
    global q4_loc
    q4_loc = data.current_pos

def cb_q5(data):
    global q5_loc
    q5_loc = data.current_pos

def cb_f(data):
    global f_loc
    f_loc = data.current_pos


def pub():
    global q0_loc, q1_loc, q2_loc, q3_loc, q4_loc, q5_loc, q6_loc, q7_loc
    rospy.init_node('enable_gravity_mode', anonymous=True)
    pub0 = rospy.Publisher('/joint_1/command', Float64, queue_size=1)
    pub1 = rospy.Publisher('/joint_2/command', Float64, queue_size=1)
    pub2 = rospy.Publisher('/joint_3/command', Float64, queue_size=1)
    pub3 = rospy.Publisher('/joint_4/command', Float64, queue_size=1)
    pub4 = rospy.Publisher('/joint_5/command', Float64, queue_size=1)
    pub5 = rospy.Publisher('/joint_6/command', Float64, queue_size=1)
    pub6 = rospy.Publisher('/finger_controller/command', Float64, queue_size=1)

    rospy.Subscriber("/joint_1/state", JointState, cb_q0)
    rospy.Subscriber("/joint_2/state", JointState, cb_q1)
    rospy.Subscriber("/joint_3/state", JointState, cb_q2)
    rospy.Subscriber("/joint_4/state", JointState, cb_q3)
    rospy.Subscriber("/joint_5/state", JointState, cb_q4)
    rospy.Subscriber("/joint_6/state", JointState, cb_q5)
    rospy.Subscriber("/finger_controller/state", JointState, cb_f)

    rospy.loginfo("START: Enabling gravity mode")

    rate = rospy.Rate(10) # 10hz
    rospy.loginfo("Getting joint data...")
    rospy.sleep(0.1)

    rospy.loginfo("Setting q0")
    pub0.publish(q0_loc)

    rospy.loginfo("Setting q1")
    pub1.publish(q1_loc)

    rospy.loginfo("Setting q2")
    pub2.publish(q2_loc)

    rospy.loginfo("Setting q3")
    pub3.publish(q3_loc)

    rospy.loginfo("Setting q4")
    pub4.publish(q4_loc)

    rospy.loginfo("Setting q5")
    pub5.publish(q5_loc)

    rospy.loginfo("Setting fingers")
    pub6.publish(f_loc)


if __name__ == '__main__':
    try:
        pub()
    except rospy.ROSInterruptException:
        pass

