#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData, Path
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
import tf
import numpy as np
from asl_turtlebot.msg import DetectedObject, DetectedObjectList, VendorLocation

x = 0
y = 0
theta = 0

vendor_location = VendorLocation()

def subscriber():

    global x, y, theta

    rospy.Subscriber('/odom', Odometry, pose_callback)

    rospy.Subscriber('/detector/stop_sign', DetectedObject, detector_callback)
    rospy.Subscriber('/detector/orange', DetectedObject, detector_callback)
    rospy.Subscriber('/detector/apple', DetectedObject, detector_callback)
    rospy.Subscriber('/detector/banana', DetectedObject, detector_callback)
    rospy.Subscriber('/detector/cat', DetectedObject, detector_callback)
    
def publisher():

    global x, y, theta, vendor_location
    rospy.init_node('turtlebot_vendorlogin', anonymous=True)
    rate = rospy.Rate(1)
    vendor_location_pub = rospy.Publisher('/vendor', VendorLocation, queue_size=10)

    
    while not rospy.is_shutdown():
    
        vendor_location_pub.publish(vendor_location)
        
        rate.sleep()
        
    
def pose_callback(data):

    global x, y, theta
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    theta = euler[2]
    # theta = -2.0 * np.arcsin(data.pose.pose.orientation.z)
    # print "x=", x, "y=", y, "theta=", theta
    
def detector_callback(msg):
    
    # print "x=", x, "y=", y, "theta=", theta
    # print "dist=", msg.distance, "thetaright=", msg.thetaright, "thetaleft=", msg.thetaleft
    
    thetaleft = msg.thetaleft - 2*np.pi if msg.thetaleft > np.pi else msg.thetaleft
    thetaright = msg.thetaright - 2*np.pi if msg.thetaright > np.pi else msg.thetaright
    theta_v = (thetaleft + thetaright) / 2.0 + theta
    dist = msg.distance
    # vendor_location.x = x + dist * np.cos(theta_v)
    # vendor_location.y = y + dist * np.sin(theta_v)
    # vendor_location.theta = theta_v
    vendor_location.x = x
    vendor_location.y = y
    vendor_location.theta = theta
    vendor_location.id = msg.id
    vendor_location.name = msg.name

if __name__== '__main__':
    try:
        subscriber()
        publisher()
    except rospy.ROSInterruptException:
        pass
