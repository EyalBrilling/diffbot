cmake_minimum_required(VERSION 3.0.2)
project(diffbot_gazebo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp gazebo_ros sensor_msgs nav_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

## Find gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} include/diffbot_gazebo)

catkin_package(
  DEPENDS gazebo
  CATKIN_DEPENDS roscpp gazebo_ros sensor_msgs nav_msgs
  INCLUDE_DIRS include
  LIBRARIES
)

add_library(gps_plugin src/odom_to_gps_plugin.cpp include/diffbot_gazebo/odom_to_gps_plugin.h)
target_link_libraries(gps_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
