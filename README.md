# Svenzva Robotics ROS drivers for ROS on Windows

This is the active repository for the Svenzva Robotics robotic product line ROS software.
The `svenzva_ros` package holds all thats needed to get up and running with ROS, including drivers, description files, simulation files and interactive utilities.

This branch is for use on a Windows 10 system with ROS on Windows installed.
The driver is not 1-for-1 when running on Linux vs Windows, i.e. some features are missing on ROS on Windows, including but not limted to: gravity compensation

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

Our testing platform is ROS on Windows on a Windows 10 machine. 
You can find out more about installing ROS on Windows [here](https://svenzva.com/installing-and-running-ros-on-windows/).

### Installing

These instructions will get you a copy of the ROS package and its dependencies on your local machine.

Clone the repository into the src folder of your workspace:
```
cd c:\catkin_ws\src

git clone https://github.com/SvenzvaRobotics/svenzva_ros.git
```
cd into the root of your workspace 
```
cd c:\catkin_ws
```
and use wstool to grab github source dependencies:
```
wstool init src PATH_TO_ROSINSTALL_FILE.rosinstall
```
where the rosinstall file desired is the __dependencies.rosinstall__ file in the root of the `svenzva_ros` package.

For now, you need to manually install MoveIt as its rosdeps don't get resolved otherwise.

```
choco upgrade ros-melodic-moveit -y
```

Finally, use rosdep to resolve dependencies specified in package descriptors:
```
rosdep install --from-paths ./ --ignore-src --rosdistro=$ROS_DISTRO -y
```

Next, install any python dependencies:
```
pip install yamlordereddictloader
pip install pandas
```
Finally, compile your workspace
```
cd ~/WORKSPACE_NAME && catkin_make
```
After successfully compiling, you may need to source your `setup.bash` file before running the `svenzva_ros` stack for the first time
```
source c:\catkin_ws\devel\setup.bat
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
