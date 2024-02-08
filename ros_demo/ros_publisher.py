#!/usr/bin/env python

##
#
# A simple ROS publisher node that publishes a dummy message
# with the current time. 
#
##

import rospy
from std_msgs.msg import Float64MultiArray

if __name__=="__main__":

    # message to publish
    x_hat_msg = Float64MultiArray()
    
    # topic name to publish to
    topic = "/x_hat"

    # initialize the node
    rospy.init_node("ros_publisher")
    pub = rospy.Publisher(topic, Float64MultiArray, queue_size=10)

    rate = rospy.Rate(100)
    start_time = rospy.get_time()

    while not rospy.is_shutdown():
        x_hat_msg.data = [rospy.get_time() - start_time, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
        pub.publish(x_hat_msg)
        rate.sleep()
