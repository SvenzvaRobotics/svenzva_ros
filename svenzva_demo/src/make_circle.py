#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2017 Svenzva Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import rospy
import rospkg
import math
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

"""
This demonstration controls the end effector to move in a circle through cartesian velocity commands
To control the end effector through cartesian velocity, the robot must be in velocity mode.

"""

def send_movement_cmds():
    global vel_pub
    i = 0
    angle_resolution = 5
    d_angle = angle_resolution*3.14/180
    angle= 0
    radius = 10
    x_center = 0
    y_center = 0
    twst_cmd = Twist()
    rospy.loginfo("Sending velocity commands.")
    while i < (360/angle_resolution):
        angle+= d_angle;
        ee_x = x_center + radius*math.cos(angle);
        ee_y = y_center + radius* math.sin(angle);
        twst_cmd.linear.x = ee_x
        twst_cmd.linear.z = ee_y
        vel_pub.publish(twst_cmd)
        rospy.loginfo(twst_cmd)
        rospy.sleep(0.05)
        i+=1
    vel_pub.publish(Twist())

def setup():
    global vel_pub
    rospy.init_node('revel_circular_eef', anonymous=False)

    vel_pub = rospy.Publisher('/revel/eef_velocity', Twist, queue_size=1)
    #joint_states_sub = rospy.Subscriber('/joint_states', JointState, js_cb, queue_size=1)

    rospy.loginfo("Setting up Revel Circle Movement program")

if __name__ == '__main__':
    try:
        setup()
        send_movement_cmds()
    except rospy.ROSInterruptException:
        pass
