#!/usr/bin/env python

##
#
# A simple ROS publisher node that publishes a dummy message
# with the current time. 
#
##

import rospy
from std_msgs.msg import Float64

if __name__=="__main__":
    topic = "/ros_to_drake"

    rospy.init_node("ros_publisher")
    pub = rospy.Publisher(topic, Float64, queue_size=10)

    rate = rospy.Rate(100)
    start_time = rospy.get_time()

    while not rospy.is_shutdown():
        msg = Float64()
        msg.data = rospy.get_time() - start_time
        pub.publish(msg)
        rate.sleep()
