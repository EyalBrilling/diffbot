#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from geographic_msgs.msg import GeoPoint
from tf.transformations import euler_from_quaternion
from math import atan2

rospy.init_node("to_point_controller")
odomSub = rospy.Subscriber("/diffbot/mobile_base_controller/odom",Odometry,odomCallback)
targetSub = rospy.Subscriber("/target_location",GeoPoint,targetCallback)
robotTwistPub = rospy.Publisher("/diffbot/mobile_base_controller/cmd_vel",Twist,queue_size=1)
rate = rospy.Rate(10)
#robotGpsLocationSub= rospy.Subscriber("/robot_location")
