#!/usr/bin/env python
import sys
import rospy
import shape_msgs.msg
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

def publish_scene_collision():
  rospy.init_node('publish_scene_collision', anonymous=True)
  collision_pub = rospy.Publisher('/collision_object', moveit_msgs.msg.CollisionObject, queue_size=1)
  ground = moveit_msgs.msg.CollisionObject()
  shape = shape_msgs.msg.SolidPrimitive()

  #Create ground plane shape
  shape.type = shape.BOX
  shape.dimensions = [1.0, 1.0, 0.01]

  #Create ground plane pose
  g_pose = geometry_msgs.msg.Pose()
  g_pose.position.z = -0.01

  #Create a ground plane
  ground.operation = ground.ADD
  ground.header.frame_id = "base_link"
  ground.id = "ground_plane"
  ground.primitives.append(shape)
  ground.primitive_poses.append(g_pose)


  #Annoyingly need to publish many times, if not spinning continuously
  collision_pub.publish(ground)
  rospy.sleep(1.0)
  collision_pub.publish(ground)
  collision_pub.publish(ground)
  collision_pub.publish(ground)
  rospy.sleep(1.0)

if __name__=='__main__':
  try:
    publish_scene_collision()
  except rospy.ROSInterruptException:
    pass
