cmake_minimum_required(VERSION 2.8.3)
project(svenzva_drivers)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    std_msgs
    message_generation
    actionlib_msgs
    kdl_parser
    orocos_kdl
    cmake_modules
    roslib
)

#kdl is cmake only. Must be separte
#find_package( orocos_kdl REQUIRED)
find_package( Eigen REQUIRED)


#find_library(orocos_kdl_LIBRARY NAMES ${orocos_kdl_LIBRARIES} PATHS
    #    ${orocos_kdl_LIBRARY_DIRS} NO_DEFAULT_PATH)
#set(orocos_kdl_LIBRARIES ${orocos_kdl_LIBRARY})

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

catkin_package(
    DEPENDS actionlib_msgs orocos_kdl roscpp
    )

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(kdl_grav src/kdl_grav.cpp)
target_link_libraries(kdl_grav ${catkin_LIBRARIES})
