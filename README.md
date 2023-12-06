# diffBot

An implementation of a robot reaching map coordinations given by a user command, using Gazebo and ROS
The package ros_control is used to move the actuators in the robot.

## Package Overview

- diff_bringup - Launch file to launch the simualtor and the controller together

- diffbot_control - control config files and implementation for ROS control used in simulation. **Branch controller_to_point_py** docs what was tried for getting to point control.

- diffbot_description - URDF description for diffbot. Heavily based on [origional diffbot](https://github.com/ros-mobile-robots/diffbot)

- diffbot_gazebo - World file for gazebo and world pluging to publish Gps coordinates calculated from odometry information.

## Quick Setup

Clone the project into you catkit_ws src folder:

```shell
cd ~/catkin_ws/src
git clone git@github.com:EyalBrilling/diffbot.git
```

Source ROS enviorment variables:

```shell
source /opt/ros/noetic/setup.bash
```

Build the project using catkin:

```shell
cd ~/catkin_ws
catkin_make
```

Source:

```shell
source devel/setup.bash
```

Now run the bringup launch file by running:

```shell
roslaunch diffbot_bringup diffbot_bringup.launch 
```

The simulator and a controller for the robot should come up on the screen.

If using a gui,In the gui folder run:

```shell
python3 main.py
```

The robot will be seen traveling in the map in correspondence to the simulator.

## Setup the project

The project uses Gazebo Citadel and ROS1 Noetic

### Setup ROS

#### Download ROS

Setup your computer to accept software from packages.ros.org:

```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Setup keys:

```shell
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

Update Debian package index:

```shell
sudo apt update
```

Install ros-noetic with desktop features:

```shell
sudo apt install ros-noetic-desktop-full
```

### Create a ROS workspace

#### Create catkin_ws folder

If you don't have a ROS workspace(catkin_ws), create one. Open a terminal and run:

```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
source devel/setup.bash
```

#### Source ROS enviorment variables

You must source this script in every bash terminal you use ROS in.

```shell
source /opt/ros/noetic/setup.bash
```

It can be convenient to automatically source this script every time a new shell is launched. These commands will do that for you.

```shell
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Download Gazebo

Make sure gazebo is installed by running:

```shell
gazebo
```

Gazebo should open on your desktop.
If not,run:

```shell
sudo apt install ros-noetic-gazebo-ros-pkgs
```

### Clone the project into the catkin folder

```shell
cd ~/catkin_ws/src
git clone git@github.com:EyalBrilling/diffbot.git
```

## Launching gazebo and ros nodes

We will launch gazebo and nodes using a launch file.

```shell
roslaunch gazebo_ros <world.launch>
```

or to launch a specific world:

```shell
roslaunch diffbot_gazebo diffbot.launch
```

## Additional resources

### Package organization

With the goal of minimazing dependencies between ROS packages, the following guidelines are used:

[https://roboticsbackend.com/package-organization-for-a-ros-stack-best-practices/](https://roboticsbackend.com/package-organization-for-a-ros-stack-best-practices/)

[https://github.com/harrisonwr/Cheatsheet/blob/master/ROS_Gazebo.md](https://github.com/harrisonwr/Cheatsheet/blob/master/ROS_Gazebo.md)

Good example:

[https://github.com/gazebosim/ros_gz_project_template](https://github.com/gazebosim/ros_gz_project_template)