cmake_minimum_required(VERSION 2.8.3)
project(svenzva_drivers)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    message_generation
    actionlib_msgs
)

add_action_files(
    DIRECTORY action
    FILES
    SvenzvaJoint.action
)
add_service_files(
    DIRECTORY srv
    FILES
    HomeArm.srv
)

generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
)

catkin_package(DEPENDS actionlib_msgs)
