# Project TODOs

## Features

- Create a **description package**.
  - URDF file for the robot. minimum 3 links and 2 joints.

- Create an **application package** for ros2 specific code.. **TBD Topics might be created as plugin in the gazebo package instend**
  - 2 topics:
    - Robot location published on `/robot_location` topic with message type `sensor_msgs/NavSatFix`
    - Target setpoint should be received in topic `/target_location` with messages of type `geographic_msgs/GeoPoint`

- Create a **Gazebo package**
  - gazebo world file, including the robot as a model(tag) and 2 topics created as plugins(tags) inside the model.
    - Guide to write new plugins:
    [https://classic.gazebosim.org/tutorials?tut=plugins_hello_world&cat=write_plugin](https://classic.gazebosim.org/tutorials?tut=plugins_hello_world&cat=write_plugin)
    - Guides to use the wrotten plugin: [https://www.youtube.com/watch?v=oHtQYPDGk3Y&ab_channel=GazeboSim](https://www.youtube.com/watch?v=oHtQYPDGk3Y&ab_channel=GazeboSim) [https://classic.gazebosim.org/tutorials?tut=ros_gzplugins](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins)

  - basic system cc and hh files.
    - Decide if PostUpdate interface heritage is enough, if not - decide on additional interfaces(e.g. Reset abd PreUpdate functionalities)

- Maybe create a seperate controller package

- Create a **Bringup package** for launch and config parameters  
  - TBD the specifics. Maybe isn't needed for the scope* of this project.

- Make sure robot starts at the geographic location with coordinates 32.072734, 34.787465 (Sarona)
  
## Bugs

## Refactoring

[-] Near the end of the project submittion, run clang-format.