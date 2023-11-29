# pinpointBot

An implementation of a robot reaching map coordinations given by a user command, using Gazebo and ROS

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
git clone git@github.com:EyalBrilling/pinpointBot.git
```

## Additional resources

### Package organization

With the goal of minimazing dependencies between ROS packages, the following guidelines are used:

[https://roboticsbackend.com/package-organization-for-a-ros-stack-best-practices/](https://roboticsbackend.com/package-organization-for-a-ros-stack-best-practices/)

[https://github.com/harrisonwr/Cheatsheet/blob/master/ROS_Gazebo.md](https://github.com/harrisonwr/Cheatsheet/blob/master/ROS_Gazebo.md)

Good example:

[https://github.com/gazebosim/ros_gz_project_template](https://github.com/gazebosim/ros_gz_project_template)