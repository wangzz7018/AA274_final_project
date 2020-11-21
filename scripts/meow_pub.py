#! /usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject,VendorLocation

publish_marker = False

def subscriber():
    global publish_marker
    #print("start subscribing about meow")
    #print(publish_marker)
    rospy.Subscriber('/vendor', VendorLocation, meow_vendor_callback)


def publisher():
    global publish_marker

    rospy.init_node('meow_node', anonymous=True)
    rate = rospy.Rate(1)
    vis_pub = rospy.Publisher('meow_topic', Marker, queue_size=1)

    while not rospy.is_shutdown():
        marker5 = Marker()

        marker5.header.frame_id = "map"
        marker5.header.stamp = rospy.Time()

        marker5.id = 5
        marker5.type = 9
        marker5.text = "Meow!Meow!Meow!!!\n" + "Check out our terminal for a joke!"
        
        marker5.pose.position.x = 0.5
        marker5.pose.position.y = 0.5
        marker5.pose.position.z = 0.5
        
        marker5.pose.orientation.x = 1.0
        marker5.pose.orientation.y = 0.0
        marker5.pose.orientation.z = 0.0
        marker5.pose.orientation.w = 1.0
        
        marker5.scale.x = 0.2
        marker5.scale.y = 0.2
        marker5.scale.z = 0.2

        if publish_marker:
            marker5.color.a = 1.0 # Don't forget to set the alpha!
            print "Muggle-Repelling Charm....Only the cleverest people can see those lines! "
        else:
            # print "setting market to be trasparent________________"
            marker5.color.a = 0.0
        
        marker5.color.r = 1.0
        marker5.color.g = 0.0
        marker5.color.b = 1.0
        
        #print("printing in subscriber!!!!!!")
        #print(publish_marker)
        

        vis_pub.publish(marker5)
        
        rate.sleep()


    

def meow_vendor_callback(data):
    global publish_marker
    #print "print data.name is ", data.name
    if data.name == "cat":
        publish_marker = True
        #print("detected cat in meow_vendor_callback")
    else:
        publish_marker = False
        #print("vendor see others")


if __name__== '__main__':
    try:
        subscriber()
        publisher()
    except rospy.ROSInterruptException:
        pass

