#! /usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject


publish_marker = False

def subscriber():
    global meow_message
    rospy.Subscriber('/detector/cat', DetectedObject, meow_callback)


def publisher():

    rospy.init_node('meow_node', anonymous=True)
    rate = rospy.Rate(1)
    vis_pub = rospy.Publisher('meow_topic', Marker, queue_size=10)

    while not rospy.is_shutdown():
        marker5 = Marker()

        marker5.header.frame_id = "map"
        marker5.header.stamp = rospy.Time()

        marker5.id = 5
        marker5.type = 9
        marker5.text = "Meow"
        
        marker5.pose.position.x = 2
        marker5.pose.position.y = 1.3
        marker5.pose.position.z = 0.2
        
        marker5.pose.orientation.x = 1.0
        marker5.pose.orientation.y = 0.0
        marker5.pose.orientation.z = 0.0
        marker5.pose.orientation.w = 1.0
        
        marker5.scale.x = 0.1
        marker5.scale.y = 0.1
        marker5.scale.z = 0.1

        marker5.color.a = 1.0 # Don't forget to set the alpha!
        marker5.color.r = 0.0
        marker5.color.g = 0.0
        marker5.color.b = 1.0
        
        print("printing")
        if publish_marker:
            vis_pub.publish(marker5)
            print('See a cat, publish meowing!')
        else:
            print("haven't detected cat")
        
        rate.sleep()


def meow_callback(data):
    publish_marker = True


if __name__== '__main__':
    try:
        subscriber()
        publisher()
    except rospy.ROSInterruptException:
        pass

