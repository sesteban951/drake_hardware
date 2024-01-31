#!/usr/bin/env python

##
#
# A simple Drake system that publishes a message to a ROS topic.
#
##

import rospy
from std_msgs.msg import Float64, String

from pydrake.all import LeafSystem, Simulator, BasicVector, DiagramBuilder

import time

# TODO: Fix the following
#       Cannot keyboard interrupt to stop the program

class RosPublisher(LeafSystem):
    """
    A Drake LeafSystem that publishes to a ROS topic at a fixed frequency.
    """
    def __init__(self):
        LeafSystem.__init__(self)

        # Declare a ROS publisher
        topic = "/ros_to_drake"
        self.ros_publisher = rospy.Publisher(topic, String, queue_size=10)

        # Declare a Drake publishing event that runs at 10 Hz
        self.DeclarePeriodicPublishEvent(
            period_sec=0.005,
            offset_sec=0.0,
            publish=self.Publish)
        
        self.t0 = time.time()
        self.c = 0

    def Publish(self, context):
        """
        Publishes a message to the ROS topic.
        """
        msg = f"The current Drake time is: {context.get_time()}" 
        self.c = self.c +1
        print("Count:               ", self.c)
        print("Time created object: ", time.time() -self.t0)
        print("Drake context time:  ", context.get_time())
        self.ros_publisher.publish(msg)

if __name__=="__main__":
    # Set up the ROS node
    rospy.init_node("drake_publisher")

    # Set up the Drake system diagram
    builder = DiagramBuilder()
    builder.AddSystem(RosPublisher())
    diagram = builder.Build()

    # Run the Drake diagram with a Simulator object
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    simulator.Initialize()
    simulator.AdvanceTo(30.0)
