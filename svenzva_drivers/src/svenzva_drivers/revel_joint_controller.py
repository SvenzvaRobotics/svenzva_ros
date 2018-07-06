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

"""
A python version of the corresponding cpp functions of the same name.
This allows this to be run as part of the overall driver, and thus have
direct access to the communications layer.

Author Maxwell Svetlik
Copyright Svenzva Robotics

"""
import rospy
import rospkg
import math
import sys
import numpy

from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import Robot
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from moveit_msgs.srv import GetStateValidity
from svenzva_drivers.svenzva_kinematics import *

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf


class RevelJointController:

    def __init__(self, controller_namespace, mx_io):
        rospy.Subscriber("/revel/eef_velocity", Twist, self.cart_vel_cb, queue_size=1);
        rospy.Subscriber("joint_states", JointState, self.js_cb, queue_size=1);
        rospack = rospkg.RosPack()
        path = rospack.get_path("svenzva_description");
        full_path = path + "/robots/svenzva_arm.urdf";

        f = file(full_path, 'r')
        self.robot = Robot.from_xml_string(f.read())
        f.close()

        self.mx_io = mx_io
        self.mNumJnts = 6
        self.gear_ratios = [4,7,7,3,4,1]
        self.js = JointState()
        self.min_limit = 20.0
        self.max_limit = 100.0
        self.last_twist = Twist()
        self.last_cmd = []
        self.last_cmd_zero = False

        self.cart_vel = Twist()
        self.arm_speed_limit = rospy.get_param('arm_speed_limit', 20.0)
        self.collision_check_enabled = rospy.get_param('collision_check_enabled', False)

        if self.collision_check_enabled:
            rospy.loginfo("MoveIt collision check ENABLED. Using environment information.")
            try:
                rospy.wait_for_service('/check_state_validity')
                self.state_validity_srv = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
            except rospy.ServiceException as exc:
                rospy.loginfo("MoveIt not. Cartesian Velocity controller will not use collision checking.")
                rospy.logerr("Cartesian collision checking DISABLED")
                self.collision_check_enabled = False
        else:
            rospy.loginfo("MoveIt collision check DISabled. Not using environment information.")
        self.group = None

        self.loop()


    def js_cb(self, msg):
        self.js = msg;

    def rad_to_raw(self, angle):
        return int(round( angle * 4096.0 / 6.2831853) )

    def radpm_to_rpm(self, rad_per_min):
        return rad_per_min * 0.159155

    def cart_vel_cb(self, msg):
        self.cart_vel = msg

    """
    Uses MoveIt to check if movement would cause collision with scene or self
    Returns true if movement causes collision
            false if movement does not cause collision
    """
    def check_if_collision(self, qdot_out, scale_factor, dt):
        rs = moveit_msgs.msg.RobotState()

        for i in range(0, self.mNumJnts):
            rs.joint_state.name.append(self.js.name[i])
            rs.joint_state.position.append(self.js.position[i] + (qdot_out[i]/scale_factor*dt))

        result = self.state_validity_srv(rs, "svenzva_arm", moveit_msgs.msg.Constraints() )
        if not result.valid:
            return True
        return False

    def send_zero_vel(self):
        tup_list = []
        for i in range(0, self.mNumJnts):
            tup_list.append( (i+1, 0))
        self.mx_io.set_multi_speed(tuple(tup_list))

    def send_last_vel(self):
        self.mx_io.set_multi_speed(tuple(self.last_cmd))

    def loop(self):
        rospy.sleep(1.0)
        count = 0
	qdot_out = [0,0,0,0,0,0]

        while not rospy.is_shutdown():
            msg = self.cart_vel
	    tup_list = [ ]

            #continues loop while no velocity messages are incoming
            if self.cart_vel == Twist():
                if self.last_cmd_zero:
                    rospy.sleep(0.1)
                    continue
                self.send_zero_vel()
                self.last_cmd_zero = True
                rospy.sleep(0.1)
                continue

	    qdot_out[0] = self.cart_vel.linear.x
	    qdot_out[1] = -1 * self.cart_vel.linear.y
	    qdot_out[2] = self.cart_vel.linear.z
	    qdot_out[3] = -1 * self.cart_vel.angular.y
	    qdot_out[4] = -1 * self.cart_vel.angular.x
	    qdot_out[5] = self.cart_vel.angular.z

            scale_factor = 1

            #check if movement causes collision, if enabled
            if self.collision_check_enabled and not self.cart_vel == Twist():
                in_collision = self.check_if_collision(qdot_out, scale_factor, 0.0075)
                if in_collision:
                    rospy.loginfo("Movement would cause collision with environment.")
                    for i in range(0, self.mNumJnts):
                        tup_list.append( (i+1, 0))
                    self.last_cmd = tup_list
                    self.last_qdot = qdot_out
                    self.mx_io.set_multi_speed(tuple(tup_list))
                    rospy.sleep(0.05)
                    continue

            for i in range(0, self.mNumJnts):
                #check if movement violates urdf joint limits
                if self.robot.joints[i+1].limit.lower >= self.js.position[i] + (qdot_out[i]/scale_factor*0.01) or self.js.position[i] + (qdot_out[i]/scale_factor*0.01) >= self.robot.joints[i+1].limit.upper:
                    rospy.logwarn("Cartesian movement would cause movement outside of joint limits. Skipping...")
                    rospy.logwarn("Movement would violate joint limit: Joint %d moving to %f with limits [%f,%f]", i+1, (qdot_out[i]/scale_factor) + self.js.position[i], self.robot.joints[i+1].limit.lower, self.robot.joints[i+1].limit.upper)
                    tup_list.append( (i+1, 0))
                else:
                    tup_list.append( (i+1, int(round(self.radpm_to_rpm(qdot_out[i] * self.gear_ratios[i] / scale_factor) / 0.229 ))))


            if len(tup_list) > 0:
                self.last_cmd = tup_list
                self.last_qdot = qdot_out
                self.mx_io.set_multi_speed(tuple(tup_list))
                self.last_cmd_zero = False
            rospy.sleep(0.05)


