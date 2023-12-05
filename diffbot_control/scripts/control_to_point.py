#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from geographic_msgs.msg import GeoPoint
from tf.transformations import euler_from_quaternion
from math import atan2

# Approximation of meters to WGS 84.(Notice it only works as the map is bounded by 500 meters)

LATITUDE_TO_METER = 111000
LONGITUDE_TO_METER = 73000

# starting robot position
xStartRobotLoc = 0.0
yStartRobotLoc = 0.0

latStartRobotLoc = 32.072734
longStartRobotLoc = 34.787465

# Robot Position and orien
robotLoc = Point(0,0,0)
theta = 0.0

latRobotLoc = 32.072734
longRoboticLoc = 34.787465

# Goal location. Starting goal is (0,0)
goal = Point(0,0,0)

def odomCallback(msg):
    global robotLoc
    global orientation

    robotLoc.x = msg.pose.pose.position.x
    robotLoc.y = msg.pose.pose.position.y
    orientation = msg.pose.pose.orientation
    (_1,_2, theta) = euler_from_quaternion([orientation.x,orientation.y,orientation.z,orientation.w]) 

def targetCallback(msg):
    global xGoal,yGoal
    xGoal,yGoal = calculateXYtarget(msg.latitude,msg.longitude)

# Calculate x and y in sim based on diff between starting and target geoPoints
def calculateXYtarget(targetLatitude,targetLongitude):
    latDiff = targetLatitude - latStartRobotLoc
    longDiff = targetLongitude - latStartRobotLoc
    targetX = latDiff * LATITUDE_TO_METER
    targetY = longDiff * LONGITUDE_TO_METER
    return targetX,targetY


rospy.init_node("to_point_controller")
odomSub = rospy.Subscriber("/diffbot/mobile_base_controller/odom",Odometry,odomCallback)
targetSub = rospy.Subscriber("/target_location",GeoPoint,targetCallback)
robotTwistPub = rospy.Publisher("/diffbot/mobile_base_controller/cmd_vel",Twist,queue_size=1)
rate = rospy.Rate(10)
#robotGpsLocationSub= rospy.Subscriber("/robot_location")
