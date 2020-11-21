#! /usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D, PoseStamped

def publish_after_subscribed():
    vis_pub = rospy.Publisher('meow_topic', Marker, queue_size=10)

    rospy.init_node('meow_node', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        marker5 = Marker()

        marker5.header.frame_id = "map"
        marker5.header.stamp = rospy.Time()

        # IMPORTANT: If you're creating multiple markers, 
        #            each need to have a separate marker ID.

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
        
        vis_pub.publish(marker5)
        
        print('Published marker!')
        
        rate.sleep()

if __name__== '__main__':
    try:
        publish_after_subscribed()
    except rospy.ROSInterruptException:
        pass

