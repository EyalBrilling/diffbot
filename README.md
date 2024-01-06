# pinpointBot

An implementation of a robot reaching map coordinations given by a user command, using Gazebo and ROS.

## Project dependencies

The project is tested on Gazebo fortress and ROS humble.
It is recommended to use those.  
[ROS humble installation](https://docs.ros.org/en/humble/Installation.html)  
[Gazebo fortress installion](https://gazebosim.org/docs/fortress/install)

In addition, the package `ros_gz` is used for a bridge between ROS and gazebo.  
[ros_gz for humble installation](https://github.com/gazebosim/ros_gz/tree/humble)

## Run simple world

Enter the src folder with the ROS2 packages

```sh
cd ~/diffbot/src
```

Build:

```sh
colcon build
```


Source:

```sh
source ./install/setup.bash 
```

Run:

```sh
ign gazebo pinpointbot_gazebo/worlds/basic_world.sdf 
```

To send command, from another terminal:

```sh
ign topic -t "model/pinpointBot/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.05}"
```