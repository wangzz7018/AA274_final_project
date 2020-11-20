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

    
    # rospy.Subscriber('/odom', Odometry, pose_callback)
    rospy.Subscriber('/detector/sink', DetectedObject, detector_callback)
    rospy.Subscriber('/detector/sports_ball', DetectedObject, detector_callback)
    # rospy.Subscriber('/detector/sink', DetectedObject, self.detector_callback)
    
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
    theta = - data.pose.pose.orientation.z * np.pi
    
def detector_callback(msg):
    
    global x, y, theta, vendor_location
    trans_listener = tf.TransformListener()
    try:
        (translation,rotation) = trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        x = translation[0]
        y = translation[1]
        euler = tf.transformations.euler_from_quaternion(rotation)
        theta = euler[2]
    except:
        pass
    print "x=", x, "y=", y, "theta=", theta
    
    thetaleft = msg.thetaleft - 2*np.pi if msg.thetaleft > np.pi else msg.thetaleft
    thetaright = msg.thetaright - 2*np.pi if msg.thetaright > np.pi else msg.thetaright
    theta_v = (thetaleft + thetaright) / 2.0 + theta
    dist = msg.distance
    vendor_location.x = x + dist * np.cos(theta_v)
    vendor_location.y = y + dist * np.sin(theta_v)
    vendor_location.theta = theta_v
    vendor_location.id = msg.id
    vendor_location.name = msg.name

if __name__== '__main__':
    try:
        subscriber()
        publisher()
    except rospy.ROSInterruptException:
        pass
