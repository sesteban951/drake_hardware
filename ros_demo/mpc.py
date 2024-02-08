#!/usr/bin/env python

##
#
# A simple Drake system that subscribes to a ROS topic and publishes to another ROS topic.
#
##

import rospy
from std_msgs.msg import Float64MultiArray

from pydrake.all import LeafSystem, Simulator, DiagramBuilder

class ROS_Drake(LeafSystem):
    """
    Simple Drake system that subscribes to a ROS topic and publishes to another ROS topic.
    """
    def __init__(self):
        LeafSystem.__init__(self)

        # Store the latest message from ROS 
        self.x_hat_msg = Float64MultiArray()
        self.mpc_trajectory_msg = Float64MultiArray()

        # ROS Subscriber and Publisher
        sub_topic = "/x_hat"
        pub_topic = "/mpc_trajectory"
        self.ros_subscriber = rospy.Subscriber(sub_topic, Float64MultiArray, self.callback)
        self.ros_publisher = rospy.Publisher(pub_topic, Float64MultiArray, queue_size=10)

        # Decalre a Drake publishing event that runs at 100 Hz
        self.DeclarePeriodicPublishEvent(
            period_sec=0.01,
            offset_sec=0.0,
            publish=self.WriteToTopic)
        
        # Decalre a Drake publishing event that runs at 100 Hz
        self.DeclarePeriodicPublishEvent(
            period_sec=0.01,
            offset_sec=0.0,
            publish=self.ReadFromTopic)
        
    def callback(self, msg):
        """
        Callback function for the ROS subscriber.
        """
        self.x_hat_msg = msg.data

    def WriteToTopic(self, context):
        """
        Publishes a message to the ROS topic.
        """
        self.mpc_trajectory_msg.data = [context.get_time(), 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
        self.ros_publisher.publish(self.mpc_trajectory_msg)

    def ReadFromTopic(self, context):
        """
        Prints the latest message from ROS.
        """
        print(f"Latest message from ROS: {self.x_hat_msg}")

if __name__=="__main__":
    
    # Set up the ROS node
    rospy.init_node("ros_drake")

    # Set up the Drae system diagram
    builder = DiagramBuilder()
    builder.AddSystem(ROS_Drake())
    diagram = builder.Build()

    # Run the Drake diagram with a Simulator object
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    simulator.Initialize()
    simulator.AdvanceTo(20.0)
