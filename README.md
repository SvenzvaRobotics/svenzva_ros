# Svenzva Robotics ROS drivers

This is the active repository for the Svenzva Robotics robotic product line ROS software.
The `svenzva_ros` package holds all thats needed to get up and running with ROS, including drivers, description files, simulation files and interactive utilities.

#### Disclaimer
This software is supplied "AS IS" without any warranties and support.
Svenzva Robotics LLC assumes no responsibility or liability for the use of the software. 
Svenzva Robotics LLC reserves the right to make changes in the software without notification. 

#### Notes on robot starting position
The Revel **must** be powered on in a particular orientation to correctly initialize the robot's motors. Generally the robot should be upright, but for the revolute joints (Joint 1, Joint 4, Joint 6) you must also consider the direction of the cable.

Here are reference photos of the Revel robot lined up to be powered on.

<img src="http://svenzva.com/wp-content/uploads/robot_starting_position.jpg" alt="Revel start position" width="250" align="middle"/>
<img src="http://svenzva.com/wp-content/uploads/revel_starting_position_2.jpg" alt="Revel start position, joint 6" width="250" align="right"/>
<img src="http://svenzva.com/wp-content/uploads/revel_starting_position_3.jpg" alt="Revel start position, joints 4 & 5" width="200" align="right"/>

Note the cord direction for the indicated joints **1a**, **1b** and **1c**. 

Prior to powering on the robot, move each joint to match that of the reference photo. Each joint should be started within ~40 degrees of the position indicated.

Failure to follow this step can cause the robot to crash into itself or the environment.


### Prerequisites

Our testing platform is ROS Indigo with 14.04 and ROS Kinetic 16.04. You must have either of these installed or  have access to a system with ROS installed to use this ROS package.
You can find out more about installing ROS [here](http://wiki.ros.org/kinetic/Installation).
The following instructions assume you have a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) setup and have [configured your environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) (Section 3) to source that workspace.

### Installing

These instructions will get you a copy of the ROS package and its dependencies on your local machine.

Clone the repository into the src folder of your workspace:
```
git clone https://github.com/SvenzvaRobotics/svenzva_ros.git
```
cd into the root of your workspace 
```
cd ~/WORKSPACE_NAME
```
and use wstool to grab github source dependencies:
```
wstool init src PATH_TO_ROSINSTALL_FILE.rosinstall
```
where the rosinstall file desired is the __dependencies.rosinstall__ file in the root of the `svenzva_ros` package.

Finally, use rosdep to resolve dependencies specified in package descriptors:
```
rosdep install --from-paths ./ --ignore-src --rosdistro=$ROS_DISTRO -y
```

Next, install any python dependencies:
```
pip install yamlordereddictloader
```
Finally, compile your workspace
```
cd ~/WORKSPACE_NAME && catkin_make
```
After successfully compiling, you may need to source your `.bashrc` file before running the `svenzva_ros` stack for the first time IF your .bashrc is configured to source the workspace
```
source ~/.bashrc
```

## Deployment

To bringup the ROS driver, bringup the main launch file:
```
roslaunch svenzva_drivers svenzva_bringup.launch
```

For additional detail on the bringing up the system, please check the [Github Wiki](https://github.com/SvenzvaRobotics/svenzva_ros/wiki).

## How to Use

Once the main launch file has been brought up, a number of robot features are exposed via ROS. 
We explore these in detail in the [Github Wiki](https://github.com/SvenzvaRobotics/svenzva_ros/wiki) pages and own support documents.

## License

This project, unless specified otherwise, is licensed under the BSD License - see the [LICENSE.md](LICENSE.md)   file for details.
