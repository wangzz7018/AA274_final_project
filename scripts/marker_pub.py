#! /usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D, PoseStamped

def publisher():
    vis_pub = rospy.Publisher('marker_topic', Marker, queue_size=10)

    rospy.init_node('marker_node', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        marker0 = Marker()
        marker1 = Marker()
        marker2 = Marker()

        marker0.header.frame_id = "map"
        marker0.header.stamp = rospy.Time()
        marker1.header.frame_id = "map"
        marker1.header.stamp = rospy.Time()
        marker2.header.frame_id = "map"
        marker2.header.stamp = rospy.Time()

        # IMPORTANT: If you're creating multiple markers, 
        #            each need to have a separate marker ID.
        marker0.id = 0
        marker1.id = 1
        marker2.id = 2

        marker0.type = 2 # sphere
        marker1.type = 2
        marker2.type = 2

        marker0.pose.position.x = 3.5
        marker0.pose.position.y = 1.6
        marker0.pose.position.z = 0.2

        marker1.pose.position.x = 2
        marker1.pose.position.y = 0.5
        marker1.pose.position.z = 0.2

        marker2.pose.position.x = 1
        marker2.pose.position.y = 2.7
        marker2.pose.position.z = 0.2

        marker0.pose.orientation.x = 0.0
        marker0.pose.orientation.y = 0.0
        marker0.pose.orientation.z = 0.0
        marker0.pose.orientation.w = 1.0

        marker1.pose.orientation.x = 0.0
        marker1.pose.orientation.y = 0.0
        marker1.pose.orientation.z = 0.0
        marker1.pose.orientation.w = 1.0

        marker2.pose.orientation.x = 0.0
        marker2.pose.orientation.y = 0.0
        marker2.pose.orientation.z = 0.0
        marker2.pose.orientation.w = 1.0

        marker0.scale.x = 0.1
        marker0.scale.y = 0.1
        marker0.scale.z = 0.1

        marker1.scale.x = 0.1
        marker1.scale.y = 0.1
        marker1.scale.z = 0.1

        marker2.scale.x = 0.1
        marker2.scale.y = 0.1
        marker2.scale.z = 0.1

        marker0.color.a = 1.0 # Don't forget to set the alpha!
        marker0.color.r = 1.0
        marker0.color.g = 0.0
        marker0.color.b = 0.0

        marker1.color.a = 1.0 # Don't forget to set the alpha!
        marker1.color.r = 0.0
        marker1.color.g = 1.0
        marker1.color.b = 0.0

        marker2.color.a = 1.0 # Don't forget to set the alpha!
        marker2.color.r = 0.0
        marker2.color.g = 0.0
        marker2.color.b = 1.0
        
        vis_pub.publish(marker0)
        vis_pub.publish(marker1)
        vis_pub.publish(marker2)

        print('Published marker!')
        
        rate.sleep()

if __name__== '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

