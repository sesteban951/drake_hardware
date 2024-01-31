#!/usr/bin/env python

##
#
# A simple Drake system that publishes a message to a ROS topic.
#
##

import rospy
from std_msgs.msg import String

from pydrake.all import LeafSystem, Simulator, BasicVector, DiagramBuilder


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
            period_sec=0.1,
            offset_sec=0.0,
            publish=self.Publish)

    def Publish(self, context):
        """
        Publishes a message to the ROS topic.
        """
        msg = f"The current Drake time is: {context.get_time()}" 
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
    simulator.AdvanceTo(10.0)
