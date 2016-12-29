#!/usr/bin/env python

"""
Author: Maxwell Svetlik
"""

import rospy
import rospkg
import actionlib
import yaml
from std_msgs.msg import Bool
from dynamixel_controllers.srv import *
from svenzva_drivers.msg import *
from svenzva_drivers.srv import *
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal


"""
Given an array of joint positions (in radians), send request to individual servos
TODO: Check if enought joint positions
      Check if motors are in joint mode and not wheel mode
"""
def fkine_action(data):

    traj_client = actionlib.SimpleActionClient('/f_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    traj_client.wait_for_server()
    rospy.loginfo("Found Trajectory action server")

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    point = JointTrajectoryPoint()
    point.positions = data.positions
    point.time_from_start = rospy.Duration(5.0)
    goal.trajectory.points.append(point)
    traj_client.send_goal_and_wait(goal)

def home_arm(data):
    # load the yaml file that specifies the home position
    rospack = rospkg.RosPack()
    path = rospack.get_path('svenzva_drivers')
    f = open(path+"/config/home_position.yaml")
    qmap = yaml.safe_load(f)
    f.close()

    #2 - execute action on yaml file
    req = SvenzvaJointGoal()
    if len(qmap['home']) < 6:
        rospy.logerr("Could not home arm. Home position configuration file ill-formed or missing. Aborting.")
        return
    req.positions = qmap['home']

    fkine = actionlib.SimpleActionClient('/svenzva_joint_action', SvenzvaJointAction)

    fkine.wait_for_server()
    rospy.loginfo("Found Trajectory action server")
    rospy.loginfo("Homing arm...")
    fkine.send_goal_and_wait(req)

def setup():
    rospy.init_node('svenzva_driver', anonymous=True)
    action = actionlib.SimpleActionServer("svenzva_joint_action", SvenzvaJointAction, fkine_action, auto_start = False)
    action.start()
    rospy.Service('home_arm_service', HomeArm, home_arm)
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        setup()
    except rospy.ROSInterruptException:
        pass

