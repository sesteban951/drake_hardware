#!/usr/bin/env python

##
#
# A simple Drake system that reads a message from a ROS topic.
#
##

import rospy
from std_msgs.msg import Float64, String

from pydrake.all import LeafSystem, Simulator, BasicVector, DiagramBuilder

# TODO: Fix the following
#       Cannot keyboard interrupt to stop the program
#       I think ROS can't keep up with the rate of the drake system
#       When you print the latest message from ROS, it shows the same value several times in a row

class RosSubscriber(LeafSystem):
    """
    A Drake LeafSystem that subscribes to a ROS topic.
    """
    def __init__(self):
        LeafSystem.__init__(self)

        # Declare a ROS subscriber
        topic = "/x_hat"
        self.ros_subscriber = rospy.Subscriber(topic, String, self.callback)

        # Store the latest message from ROS
        self.latest_message = None

        # Declare a periodic publish event that runs at 10 Hz. In this case
        # this event has nothing to do with publishing, but we use it to
        # just print out the latest message from ROS. In practice this would be
        # replaced with output ports or something more interesting. 
        self.DeclarePeriodicPublishEvent(
            period_sec=0.01,
            offset_sec=0.0,
            publish=self.PrintMessage)

        # Spin for a bit to let the subscriber connect
        rospy.sleep(1)

    def PrintMessage(self, context):
        """
        Prints the latest message from ROS.
        """
        print(f"Latest message from ROS: {self.latest_message}")

    def callback(self, msg):
        """
        Callback function for the ROS subscriber.
        """
        self.latest_message = msg.data

if __name__=="__main__":
    # Set up the ROS node
    rospy.init_node("drake_subscriber")

    # Set up the Drake system diagram
    builder = DiagramBuilder()
    builder.AddSystem(RosSubscriber())
    diagram = builder.Build()

    # Run the Drake diagram with a Simulator object
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    simulator.Initialize()
    simulator.AdvanceTo(40.0)
