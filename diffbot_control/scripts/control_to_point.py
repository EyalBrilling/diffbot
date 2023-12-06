#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from geographic_msgs.msg import GeoPoint
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion
from math import atan2

# Approximation of meters to WGS 84.(Notice it only works as the map is bounded by 500 meters)

LATITUDE_TO_METER = 111000
LONGITUDE_TO_METER = 73000

TARGET_STOP_RANGE_METER = 1

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

def modelStatesCallback(modelState):
    global robotLoc
    global theta
    robotLoc.x = modelState.pose.position.x
    robotLoc.y = modelState.pose.position.y
    orientation = modelState.pose.orientation
    (roll,pitch, theta) = euler_from_quaternion([orientation.x,orientation.y,orientation.z,orientation.w]) 
    
def targetCallback(msg):
    global goal
    goal.x,goal.y = calculateXYtarget(msg.latitude,msg.longitude)


# Calculate x and y in sim based on diff between starting and target geoPoints
def calculateXYtarget(targetLatitude,targetLongitude):
    latDiff = (targetLatitude - latStartRobotLoc) / 10
    longDiff = (targetLongitude - longStartRobotLoc) / 10
    
    targetX = latDiff * LATITUDE_TO_METER
    targetY = longDiff * LONGITUDE_TO_METER
    print("targetLatitude",targetLatitude)
    print("targetLongitude",targetLongitude)
    print("targetX:",targetX)
    print("targetY:",targetY)
    return targetX,targetY


rospy.init_node("to_point_controller")
rospy.wait_for_service('/gazebo/get_model_state')
modelStatesService = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
targetSub = rospy.Subscriber("/target_location",GeoPoint,targetCallback)
robotTwistPub = rospy.Publisher("/diffbot/mobile_base_controller/cmd_vel",Twist,queue_size=1)
rate = rospy.Rate(100)
 
robotTwist = Twist()

while not rospy.is_shutdown():
    modelState =modelStatesService("diffbot","")
    modelStatesCallback(modelState)
    xDiffRobotTarget = goal.x - robotLoc.x
    yDiffRobotTarget = goal.y - robotLoc.y
    angle_to_goal = atan2(yDiffRobotTarget , xDiffRobotTarget)
    if abs(angle_to_goal - theta) > 0.1:
        robotTwist.linear.x = 0.0
        robotTwist.angular.z = 0.5
    else:
        robotTwist.linear.x = 0.5
        robotTwist.angular.z = 0.0
    robotTwistPub.publish(robotTwist)
    rate.sleep()