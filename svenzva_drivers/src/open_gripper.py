#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState

def callback(data):
    global finger_load
    finger_load = data.load

def talker():
    pub = rospy.Publisher('/finger_controller/command', Float64, queue_size=1)
    rospy.init_node('gripper_demo_open', anonymous=True)
    rospy.Subscriber("/finger_controller/state", JointState, callback)
    global finger_load
    finger_load = 0.0
    rate = rospy.Rate(20) # 10hz
    torque_cutoff = 0.25 #[0,1] and must be lower than the max torque value set for the servos
    #vel = 0.05
    pos = 3.14
    hit_cutoff = False
    cmd = Float64()
    cmd.data = pos
    pub.publish(cmd)
    rospy.sleep(0.5)
    """
    while not rospy.is_shutdown() and not hit_cutoff:
        rospy.loginfo("Load: %f", finger_load)
        if finger_load >= torque_cutoff:
            hit_cutoff = True
            cmd.data = vel
        pub.publish(cmd)
        rate.sleep()
    rospy.sleep(0.5)
    cmd.data = 0.0
    pub.publish(cmd)
    """
    rospy.loginfo("Exiting node. Gripper should be open.")

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
